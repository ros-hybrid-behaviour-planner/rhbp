#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: rieger
'''
import time
import unittest

import rospy
import rostest
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.msg import Fact, FactRemoved, FactUpdated
from knowledge_base.srv import All

PKG = 'knowledge_base'

"""
System test for knowledge base. Assumes, that a rosmaster and the knowledge base is running
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

        self.add_msg = None
        self.remove_msg = None
        self.update_msg = None

    def __handle_add_update(self, msg):
        self.add_msg = msg

    def __handle_remove_update(self, msg):
        self.remove_msg = msg

    def __handle_fact_update(self, msg):
        self.update_msg = msg

    def is_empty(self):
        return self.update_msg is None and self.remove_msg is None and self.add_msg is None

    def clear(self):
        self.add_msg = None
        self.remove_msg = None
        self.update_msg = None


class TupleSpaceTestSuite(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TupleSpaceTestSuite, self).__init__(*args, **kwargs)
        rospy.init_node('TupleSpaceTestSuite', log_level=rospy.DEBUG)
        # prevent influence of previous tests
        self.__message_prefix = 'TupleSpaceTestSuite' + str(time.time())
        self.__knowledge_base_address = KnowledgeBase.DEFAULT_NAME
        self.__client = KnowledgeBaseClient(self.__knowledge_base_address)

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
        self.assertEqual(1,len(removed))
        self.assertEqual(test_tuple, removed[0])

        self.assertFalse(self.__client.exists(test_tuple))

    def __add_multiple_facts(self, facts):
        for t in facts:
            self.__client.push(t)
        self.__wait_for_tuple(facts[len(facts)-1])

    def __test_list_equality(self,l1,l2):
        error_message = 'Excepted "{0}", but is {1}'.format(str(l1),str(l2))
        self.assertEqual(len(l1),len(l2))
        for i1 in l1:
            self.assertTrue(i1 in l2,error_message)
        for i2 in l2:
            self.assertTrue(i2 in l1,error_message)

    def test_remove_of_non_existing(self):
        tuples = []
        tuples.append((self.__message_prefix,'test_remove_of_non_existing','0','1'))
        tuples.append((self.__message_prefix,'test_remove_of_non_existing','2','5'))
        self.__add_multiple_facts(tuples)

        self.__check_content((self.__message_prefix,'test_remove_of_non_existing','*','*'),tuples[0],tuples[1])
        removed = self.__client.pop((self.__message_prefix,'test_remove_of_non_existing','4','*'))
        self.assertEqual(0,len(removed))
        self.__check_content((self.__message_prefix,'test_remove_of_non_existing','*','*'),tuples[0],tuples[1])

    def test_multiple_pop(self):
        uninteresting_content = []
        uninteresting_content.append((self.__message_prefix,'test_multiple_pop','boring','1'))
        uninteresting_content.append((self.__message_prefix,'test_multiple_pop','boring','5'))
        to_remove = []
        to_remove.append((self.__message_prefix,'test_multiple_pop','interesting','1'))
        to_remove.append((self.__message_prefix,'test_multiple_pop','interesting','5'))
        all = []
        all.extend(uninteresting_content)
        all.extend(to_remove)
        self.__add_multiple_facts(all)
        self.__check_content((self.__message_prefix,'test_multiple_pop','*','*'),*tuple(all))
        removed = self.__client.pop((self.__message_prefix,'test_multiple_pop','interesting','*'))
        self.__test_list_equality(removed,to_remove)
        self.__check_content((self.__message_prefix,'test_multiple_pop','*','*'),*tuple(uninteresting_content))


    def test_placeholder(self):
        test_tuple = (self.__message_prefix, 'test_placeholder', '0', '0')
        pattern = (self.__message_prefix, 'test_placeholder', '*', '*')

        self.__client.push(test_tuple)
        self.assertEqual(test_tuple, self.__client.peek(pattern))

    def __wait_for_tuple(self, wait_for_it):
        """
        waits, until the requested tuple is contained in knowledge_base
        :param wait_for_it: tuple
        """
        while not self.__client.exists(wait_for_it):
            rospy.sleep(0.1)

    @staticmethod
    def __is_tuple_in_facts(to_check, facts):
        """
        :param to_check: tuple
        :param facts: tuple of ROS message Fact
        :return: whether tuple is contained in content of at least one fact
        """
        for current_tuple in facts:
            if tuple(current_tuple.content) == to_check:
                return True
        return False

    def test_all(self):
        """
        test service for find all matching facts
        """
        tuples = []
        tuples.append((self.__message_prefix, 'test_all', 'pos', '0', '0'))
        tuples.append((self.__message_prefix, 'test_all', 'pos', '1', '0'))
        tuples.append((self.__message_prefix, 'test_all', 'pos', '1', '-4'))
        self.__add_multiple_facts(tuples)

        self.__check_content((self.__message_prefix, 'test_all', 'pos', '*', '*'),*tuple(tuples))

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
            self.assertIsNone(update_mock.add_msg,
                              'Following mock has an unexpected add message received: ' + update_mock.name)
            self.assertIsNone(update_mock.remove_msg,
                              'Following mock has an unexpected remove message received: ' + update_mock.name)
            self.assertIsNone(update_mock.update_msg,
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

        self.assertFalse(self.__client.update((self.__message_prefix, 'unsuccess', '1'), new_fact),
                         'Old fact does not exist, but update was successfull')
        rospy.sleep(0.1)
        self.__check_mocks_empty(all_informers)
        self.__check_content((prefix, '*', '*'), not_influenced)

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
        informer_seccond_target = UpdateSubscriberMock(mock_name='seccond', client=self.__client,
                                                       pattern=(prefix, 'seccond_target', '*'))
        informer_not_influenced = UpdateSubscriberMock(mock_name='not_influended', client=self.__client,
                                                       pattern=(prefix, 'not_influenced', '*'))
        all_informers = [informer_added, informer_removed, informer_updated, informer_seccond_target,
                         informer_not_influenced]

        self.assertTrue(self.__client.update(old_fact, new_fact), 'First update failed')
        rospy.sleep(0.1)

        self.assertIsNotNone(informer_added.add_msg, 'added message was not received')
        self.assertEqual(new_fact, tuple(informer_added.add_msg.content))
        informer_added.add_msg = None

        self.assertIsNotNone(informer_removed.remove_msg, 'remove message was not received')
        self.assertEqual(old_fact, tuple(informer_removed.remove_msg.fact))
        informer_removed.remove_msg = None

        self.assertIsNotNone(informer_updated.update_msg, 'update message was not received')
        self.assertEqual(old_fact, tuple(informer_updated.update_msg.old))
        self.assertEqual(new_fact, tuple(informer_updated.update_msg.new))
        informer_updated.update_msg = None

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

        self.__client.push(old_fact)
        self.__client.push(new_fact)
        self.__client.push(not_influenced)
        self.__check_content((prefix, '*', '*'), old_fact, new_fact, not_influenced)

        informer_added = UpdateSubscriberMock(mock_name='added', client=self.__client, pattern=(prefix, 'new', '*'))
        informer_removed = UpdateSubscriberMock(mock_name='removed', client=self.__client, pattern=(prefix, 'old', '*'))
        informer_updated = UpdateSubscriberMock(mock_name='updated', client=self.__client, pattern=(prefix, '*', '*'))
        informer_seccond_target = UpdateSubscriberMock(mock_name='seccond', client=self.__client,
                                                       pattern=(prefix, 'seccond_target', '*'))
        informer_not_influenced = UpdateSubscriberMock(mock_name='not_influended', client=self.__client,
                                                       pattern=(prefix, 'not_influenced', '*'))
        all_informers = [informer_added, informer_removed, informer_updated, informer_seccond_target,
                         informer_not_influenced]

        self.assertTrue(self.__client.update(old_fact, new_fact), 'update failed')
        rospy.sleep(0.1)

        self.assertIsNotNone(informer_removed.remove_msg, 'remove message was not received')
        self.assertEqual(old_fact, tuple(informer_removed.remove_msg.fact))
        informer_removed.remove_msg = None
        self.assertIsNotNone(informer_updated.remove_msg, 'seccond remove message was not received')
        self.assertEqual(old_fact, tuple(informer_updated.remove_msg.fact))
        informer_updated.remove_msg = None

        self.__check_mocks_empty(all_informers)
        self.__check_content((prefix, '*', '*'), new_fact, not_influenced)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'knowledge_base_manager_test_node', TupleSpaceTestSuite)
