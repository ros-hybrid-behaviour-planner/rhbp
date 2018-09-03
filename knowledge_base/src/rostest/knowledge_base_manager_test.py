#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: rieger
'''
import time
import unittest

import rospy
import rostest
import roslaunch

from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.msg import Fact, FactRemoved, FactUpdated
from knowledge_base.srv import All

PKG = 'knowledge_base'

"""
System test for knowledge base.
"""


class UpdateSubscriberMock(object):
    """
    Subscribes to the knowledge base and record received message
    """

    def __init__(self, mock_name, pattern, client):
        self.name = mock_name
        added_topic_name, update_topic_name, removed_topic_name = client.subscribe_for_updates(pattern)
        rospy.Subscriber(added_topic_name, Fact, self.__handle_add_update)
        rospy.Subscriber(removed_topic_name, FactRemoved, self.__handle_remove_update)
        rospy.Subscriber(update_topic_name, FactUpdated, self.__handle_fact_update)

        self.add_msg = []
        self.remove_msg = []
        self.update_msg = []

    def __handle_add_update(self, msg):
        self.add_msg.append(msg)

    def __handle_remove_update(self, msg):
        self.remove_msg.append(msg)

    def __handle_fact_update(self, msg):
        self.update_msg.append(msg)

    def is_empty(self):
        return not (self.update_msg or self.remove_msg or self.add_msg)

    def clear(self):
        self.add_msg = []
        self.remove_msg = []
        self.update_msg = []


class TupleSpaceTestSuite(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TupleSpaceTestSuite, self).__init__(*args, **kwargs)

    def setUp(self):

        # prevent influence of previous tests
        self.__message_prefix = 'TupleSpaceTestSuite' + str(time.time())
        self.__knowledge_base_address = KnowledgeBase.DEFAULT_NAME

        #start KnowledgeBase
        package = 'knowledge_base'
        executable = 'knowledge_base_node.py'
        node = roslaunch.core.Node(package=package, node_type=executable, name=self.__knowledge_base_address)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        self._kb_process = launch.launch(node)

        rospy.init_node('TupleSpaceTestSuite', log_level=rospy.DEBUG)

        self.__client = KnowledgeBaseClient(self.__knowledge_base_address)

    def tearDown(self):
        self._kb_process.stop()

    def __wait_for_tuple(self, wait_for_it):
        """
        waits, until the requested tuple is contained in knowledge_base
        :param wait_for_it: tuple
        """
        while not self.__client.exists(wait_for_it):
            rospy.sleep(0.1)

    def __add_multiple_facts(self, *facts):
        for t in facts:
            self.__client.push(t)
        self.__wait_for_tuple(facts[len(facts) - 1])

    def __test_list_equality(self, l1, l2):
        error_message = 'Excepted "{0}", but is {1}'.format(str(l1), str(l2))
        self.assertEqual(len(l1), len(l2))
        for i1 in l1:
            self.assertTrue(i1 in l2, error_message)
        for i2 in l2:
            self.assertTrue(i2 in l1, error_message)

    def test_exists_for_non_existing(self):
        test_tuple = (self.__message_prefix, 'test_exists_for_non_existing', '0', '0')
        self.assertFalse(self.__client.exists(test_tuple))

    def test_simple_adding(self):
        test_tuple = (self.__message_prefix, 'test_simple_adding', '0', '0')
        self.__client.push(test_tuple)
        self.assertTrue(self.__client.exists(test_tuple))

    def test_peek(self):
        test_tuple = (self.__message_prefix, 'test_peek', '0', '0')
        self.__client.push(test_tuple)
        self.assertEqual(test_tuple, self.__client.peek(test_tuple))

    def test_pop(self):
        test_tuple = (self.__message_prefix, 'test_pop', '0', '0')
        self.__client.push(test_tuple)
        self.assertTrue(self.__client.exists(test_tuple))

        removed = self.__client.pop(test_tuple)
        self.assertEqual(1, len(removed))
        self.assertEqual(test_tuple, removed[0])

    def test_remove_of_non_existing(self):
        tuples = []
        tuples.append((self.__message_prefix, 'test_remove_of_non_existing', '0', '1'))
        tuples.append((self.__message_prefix, 'test_remove_of_non_existing', '2', '5'))
        self.__add_multiple_facts(*tuples)

        self.__check_content((self.__message_prefix, 'test_remove_of_non_existing', '*', '*'), *tuples)
        removed = self.__client.pop((self.__message_prefix, 'test_remove_of_non_existing', '4', '*'))
        self.assertEqual(0, len(removed))
        self.__check_content((self.__message_prefix, 'test_remove_of_non_existing', '*', '*'), *tuples)

    def test_multiple_pop(self):
        uninteresting_content = []
        uninteresting_content.append((self.__message_prefix, 'test_multiple_pop', 'boring', '1'))
        uninteresting_content.append((self.__message_prefix, 'test_multiple_pop', 'boring', '5'))
        to_remove = []
        to_remove.append((self.__message_prefix, 'test_multiple_pop', 'interesting', '1'))
        to_remove.append((self.__message_prefix, 'test_multiple_pop', 'interesting', '5'))
        all = []
        all.extend(uninteresting_content)
        all.extend(to_remove)
        self.__add_multiple_facts(*all)
        self.__check_content((self.__message_prefix, 'test_multiple_pop', '*', '*'), *tuple(all))
        removed = self.__client.pop((self.__message_prefix, 'test_multiple_pop', 'interesting', '*'))
        self.__test_list_equality(removed, to_remove)
        self.__check_content((self.__message_prefix, 'test_multiple_pop', '*', '*'), *tuple(uninteresting_content))

    def test_placeholder(self):
        test_tuple = (self.__message_prefix, 'test_placeholder', '0', '0')
        pattern = (self.__message_prefix, 'test_placeholder', '*', '*')

        self.__client.push(test_tuple)
        self.assertEqual(test_tuple, self.__client.peek(pattern))

    def test_all(self):
        """
        test service for find all matching facts
        """
        tuples = []
        tuples.append((self.__message_prefix, 'test_all', 'pos', '0', '0'))
        tuples.append((self.__message_prefix, 'test_all', 'pos', '1', '0'))
        tuples.append((self.__message_prefix, 'test_all', 'pos', '1', '-4'))
        self.__add_multiple_facts(*tuples)

        self.__check_content((self.__message_prefix, 'test_all', 'pos', '*', '*'), *tuple(tuples))

    def __check_content(self, pattern, *expected):
        """
        ensures, that exact the expected content is contained in the knowledge base
        :param pattern: filter for deciding, which contained facts of the kb should used
        :param expected: expected content of the kb
        """
        knowledge_base_content = self.__client.all(pattern)
        self.assertEqual(len(expected), len(knowledge_base_content),
                         ' Expected: ' + str(expected) + ' but is ' + str(knowledge_base_content))
        for fact in expected:
            self.assertTrue(fact in knowledge_base_content,
                            ' Expected: ' + str(expected) + ' but is ' + str(knowledge_base_content))

    def test_prevent_multiple_adding(self):
        """
        tests that no duplicates are contained in knowledge base
        """
        test_tuple = (self.__message_prefix, 'test_prevent_multiple_adding', '0', '0')
        self.__client.push(test_tuple)
        self.__client.push(test_tuple)

        service_name = self.__knowledge_base_address + KnowledgeBase.ALL_SERVICE_NAME_POSTFIX
        rospy.wait_for_service(service_name)
        all_service = rospy.ServiceProxy(service_name, All)
        all_response = all_service(test_tuple)
        self.assertEqual(1, len(all_response.found))

    def __check_mocks_empty(self, update_mocks):
        """
        Asserts, that all given mocks has no remaining messages. All expected messages must be removed before method call
        :param update_mocks (list of UpdateSubscriberMock)
        """
        for update_mock in update_mocks:
            self.assertTrue(len(update_mock.add_msg) == 0,
                            'Following mock has an unexpected add message received: ' + update_mock.name)
            self.assertTrue(len(update_mock.remove_msg) == 0,
                            'Following mock has an unexpected remove message received: ' + update_mock.name)
            self.assertTrue(len(update_mock.update_msg) == 0,
                            'Following mock has an unexpected update message received: ' + update_mock.name)

    def test_update_non_existing(self):
        """
        Tries to update a fact, which is not in the kb
        """
        prefix = self.__message_prefix + '_test_update_non_existing'
        new_fact = (prefix, 'new', '1')
        not_influenced = (prefix, 'not_influenced', '1')

        self.__client.push(not_influenced)

        informer_added = UpdateSubscriberMock(mock_name='added', client=self.__client,
                                              pattern=(prefix, 'new', '*'))
        informer_removed = UpdateSubscriberMock(mock_name='removed', client=self.__client,
                                                pattern=(prefix, 'unsuccess', '*'))
        informer_updated = UpdateSubscriberMock(mock_name='updated', client=self.__client,
                                                pattern=(prefix, '*', '*'))
        informer_not_influenced = UpdateSubscriberMock(mock_name='not_influended', client=self.__client,
                                                       pattern=(prefix, 'not_influenced', '*'))
        all_informers = [informer_added, informer_removed, informer_updated, informer_not_influenced]

        self.assertFalse(
            self.__client.update((self.__message_prefix, 'unsuccess', '1'), new_fact, push_without_existing=False),
            'Old fact does not exist, but update was successful')
        rospy.sleep(0.5)
        self.__check_mocks_empty(all_informers)
        self.__check_content((prefix, '*', '*'), not_influenced)

        self.assertTrue(
            self.__client.update((self.__message_prefix, 'not_existing', '1'), new_fact, push_without_existing=True),
            'Old fact does not exists and update was not successful')
        rospy.sleep(0.5)

        self.assertEqual(1, len(informer_added.add_msg),
                         'Error at receiving add message: ' + str(len(informer_added.add_msg)))
        self.assertEqual(new_fact, tuple(informer_added.add_msg[0].content))
        informer_added.add_msg = []

        self.assertEqual(1, len(informer_updated.add_msg),
                         'Error at receiving add message: ' + str(len(informer_updated.add_msg)))
        self.assertEqual(new_fact, tuple(informer_updated.add_msg[0].content))
        informer_updated.add_msg = []

        self.__check_mocks_empty(all_informers)
        self.__check_content((prefix, '*', '*'), not_influenced, new_fact)

    def test_update_basic(self):
        """
        Tests simple update of an existing fact
        """
        prefix = self.__message_prefix + '_test_update_basic'
        old_fact = (prefix, 'old', '2')
        new_fact = (prefix, 'new', '1')
        not_influenced = (prefix, 'not_influenced', '1')

        self.__client.push(old_fact)
        self.__client.push(not_influenced)
        self.__check_content((prefix, '*', '*'), old_fact, not_influenced)

        informer_added = UpdateSubscriberMock(mock_name='added', client=self.__client,
                                              pattern=(prefix, 'new', '*'))
        informer_removed = UpdateSubscriberMock(mock_name='removed', client=self.__client,
                                                pattern=(prefix, 'old', '*'))
        informer_updated = UpdateSubscriberMock(mock_name='updated', client=self.__client,
                                                pattern=(prefix, '*', '*'))
        informer_second_target = UpdateSubscriberMock(mock_name='seccond', client=self.__client,
                                                       pattern=(prefix, 'seccond_target', '*'))
        informer_not_influenced = UpdateSubscriberMock(mock_name='not_influended', client=self.__client,
                                                       pattern=(prefix, 'not_influenced', '*'))
        all_informers = [informer_added, informer_removed, informer_updated, informer_second_target,
                         informer_not_influenced]

        self.assertTrue(self.__client.update(old_fact, new_fact), 'First update failed')
        rospy.sleep(0.1)

        self.assertEqual(1, len(informer_added.add_msg),
                         'Error at receiving add message: ' + str(len(informer_added.add_msg)))
        self.assertEqual(new_fact, tuple(informer_added.add_msg[0].content))
        informer_added.add_msg = []

        self.assertEqual(1, len(informer_removed.remove_msg),
                         'Error at receiving remove message: ' + str(len(informer_removed.remove_msg)))
        self.assertEqual(old_fact, tuple(informer_removed.remove_msg[0].fact))
        informer_removed.remove_msg = []

        self.assertEqual(1, len(informer_updated.update_msg),
                         'Error at receiving update message: ' + str(len(informer_updated.update_msg)))
        self.assertEqual(1, len(informer_updated.update_msg[0].removed))
        self.assertEqual(old_fact, tuple(informer_updated.update_msg[0].removed[0].content))
        self.assertEqual(new_fact, tuple(informer_updated.update_msg[0].new))
        informer_updated.update_msg = []

        self.__check_mocks_empty(all_informers)
        self.__check_content((prefix, '*', '*'), new_fact, not_influenced)

    def test_update_existing_target(self):
        """
        Tests update of an existing fact to a fact, which is already contained in the kb
        """
        prefix = self.__message_prefix + '_test_update_existing_target'
        old_fact = (prefix, 'old', '2')
        new_fact = (prefix, 'new', '1')
        not_influenced = (prefix, 'not_influenced', '1')

        self.__add_multiple_facts(old_fact, new_fact, not_influenced)
        self.__check_content((prefix, '*', '*'), old_fact, new_fact, not_influenced)

        informer_added = UpdateSubscriberMock(mock_name='added', client=self.__client, pattern=(prefix, 'new', '*'))
        informer_removed = UpdateSubscriberMock(mock_name='removed', client=self.__client, pattern=(prefix, 'old', '*'))
        informer_updated = UpdateSubscriberMock(mock_name='updated', client=self.__client, pattern=(prefix, '*', '*'))
        informer_second_target = UpdateSubscriberMock(mock_name='second', client=self.__client,
                                                      pattern=(prefix, 'second_target', '*'))
        informer_not_influenced = UpdateSubscriberMock(mock_name='not_influenced', client=self.__client,
                                                       pattern=(prefix, 'not_influenced', '*'))
        all_informers = [informer_added, informer_removed, informer_updated, informer_second_target,
                         informer_not_influenced]

        self.assertTrue(self.__client.update(old_fact, new_fact), 'update failed')
        rospy.sleep(0.1)

        self.assertEqual(1, len(informer_removed.remove_msg),
                         'Error at receiving remove message: ' + str(len(informer_removed.remove_msg)))
        self.assertEqual(old_fact, tuple(informer_removed.remove_msg[0].fact))
        informer_removed.remove_msg = []

        self.assertEqual(1, len(informer_updated.update_msg),
                         'Error at receiving update message: ' + str(len(informer_updated.update_msg)))
        self.assertEqual(old_fact, tuple(informer_updated.update_msg[0].removed[0].content))
        informer_updated.update_msg = []

        self.assertEqual(1, len(informer_added.add_msg),
                         'Error at receiving add message: ' + str(len(informer_added.add_msg)))
        self.assertEqual(new_fact, tuple(informer_added.add_msg[0].content))
        informer_added.add_msg = []

        self.__check_mocks_empty(all_informers)
        self.__check_content((prefix, '*', '*'), new_fact, not_influenced)

    def test_replace_several_facts(self):
        prefix = self.__message_prefix + '_test_replace_several_facts'
        old_fact_1 = (prefix, 'old', '2')
        old_fact_2 = (prefix, 'old', '3')
        not_influenced = (prefix, 'not_influenced', '1')
        self.__add_multiple_facts(old_fact_1, old_fact_2, not_influenced)
        self.__check_content((prefix, '*', '*'), old_fact_1, old_fact_2, not_influenced)

        new_fact = (prefix, 'new', '1')

        informer_added = UpdateSubscriberMock(mock_name='added', client=self.__client, pattern=(prefix, 'new', '*'))
        informer_removed = UpdateSubscriberMock(mock_name='removed', client=self.__client, pattern=(prefix, 'old', '*'))
        informer_updated = UpdateSubscriberMock(mock_name='updated', client=self.__client, pattern=(prefix, '*', '*'))
        informer_not_influenced = UpdateSubscriberMock(mock_name='not_influended', client=self.__client,
                                                       pattern=(prefix, 'not_influenced', '*'))
        all_informers = [informer_added, informer_removed, informer_updated, informer_not_influenced]

        self.assertTrue(self.__client.update((prefix, 'old', '*'), new_fact), 'update failed')
        rospy.sleep(0.1)

        self.__check_content((prefix, '*', '*'), new_fact, not_influenced)

        self.assertEqual(1, len(informer_added.add_msg),
                         'Error at receiving add message: ' + str(len(informer_added.add_msg)))
        self.assertEqual(new_fact, tuple(informer_added.add_msg[0].content))
        informer_added.add_msg = []

        self.assertEqual(2, len(informer_removed.remove_msg),
                         'Error at receiving remove messages: ' + str(len(informer_removed.remove_msg)))
        removed_facts = [old_fact_1, old_fact_2]
        for remove_msg in informer_removed.remove_msg:
            self.assertTrue(tuple(remove_msg.fact) in removed_facts,
                            str(remove_msg.fact) + ' not in ' + str(removed_facts))
            removed_facts.remove(tuple(remove_msg.fact))
        informer_removed.remove_msg = []

        self.assertEqual(1, len(informer_updated.update_msg),
                         'Error at receiving update message: ' + str(len(informer_updated.update_msg)))
        self.assertEqual(tuple(informer_updated.update_msg[0].new), new_fact)
        removed_facts = [old_fact_1, old_fact_2]
        for removed_fact in informer_updated.update_msg[0].removed:
            self.assertTrue(tuple(removed_fact.content) in removed_facts,
                            str(tuple(removed_fact.content)) + ' not in ' + str(removed_facts))
            removed_facts.remove(tuple(removed_fact.content))
        informer_updated.update_msg = []

        self.__check_mocks_empty(all_informers)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'knowledge_base_manager_test_node', TupleSpaceTestSuite)
