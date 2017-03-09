#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: rieger
'''
import time
import unittest

import rospy
import rostest
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.msg import Push, Fact, FactRemoved, FactUpdated
from knowledge_base.srv import Exists, Peek, Pop, All, UpdateSubscribe, Update, UpdateRequest

PKG = 'knowledge_base'

"""
System test for knowledge base. Assumes, that a rosmaster and the knowledge base is running
"""


class UpdateSubscriberMock(object):
    def __init__(self, mock_name, pattern, knowledge_base_name):
        register_for_updates_services = rospy.ServiceProxy(
            knowledge_base_name + KnowledgeBase.UPDATE_SUBSCRIBER_NAME_POSTFIX,
            UpdateSubscribe)
        self.name = mock_name
        response = register_for_updates_services(pattern)
        rospy.Subscriber(response.add_topic_name, Fact, self.__handle_add_update)
        rospy.Subscriber(response.remove_topic_name, FactRemoved, self.__handle_remove_update)
        rospy.Subscriber(response.updated_topic_name, FactUpdated, self.__handle_fact_update)
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
        self.__exists_service_name = self.__knowledge_base_address + KnowledgeBase.EXISTS_SERVICE_NAME_POSTFIX
        self.__peek_service_name = self.__knowledge_base_address + KnowledgeBase.PEEK_SERVICE_NAME_POSTFIX
        self.__all_service_name = self.__knowledge_base_address + KnowledgeBase.ALL_SERVICE_NAME_POSTFIX
        self.__pop_service_name = self.__knowledge_base_address + KnowledgeBase.POP_SERVICE_NAME_POSTFIX
        self.__update_service_name = self.__knowledge_base_address + KnowledgeBase.UPDATE_SERVICE_NAME_POSTFIX

    @staticmethod
    def add_tuple(*to_add):
        pub = rospy.Publisher(KnowledgeBase.DEFAULT_NAME + KnowledgeBase.PUSH_TOPIC_NAME_POSTFIX, Push, queue_size=10)
        rospy.sleep(0.1)
        for fact in to_add:
            pub.publish(fact)

    def test_exists_for_non_existing(self):
        test_tuple = (self.__message_prefix, 'test_exists_for_non_existing', '0', '0')
        rospy.wait_for_service(self.__exists_service_name)
        exist_service = rospy.ServiceProxy(self.__exists_service_name, Exists)
        self.assertFalse(exist_service(test_tuple).exists)

    def test_simple_adding(self):
        test_tuple = (self.__message_prefix, 'test_simple_adding', '0', '0')
        self.add_tuple(test_tuple)
        rospy.wait_for_service(self.__exists_service_name)
        exist_service = rospy.ServiceProxy(self.__exists_service_name, Exists)
        self.assertTrue(exist_service(test_tuple))

    def test_peek(self):
        test_tuple = (self.__message_prefix, 'test_peek', '0', '0')
        self.add_tuple(test_tuple)
        rospy.wait_for_service(self.__peek_service_name)
        peek_service = rospy.ServiceProxy(self.__peek_service_name, Peek)
        peek_response = peek_service(test_tuple)
        self.assertTrue(True, peek_response.exists)
        self.assertEqual(test_tuple, tuple(peek_response.example))

    def test_pop(self):
        test_tuple = (self.__message_prefix, 'test_pop', '0', '0')
        self.add_tuple(test_tuple)
        rospy.wait_for_service(self.__exists_service_name)
        exist_service = rospy.ServiceProxy(self.__exists_service_name, Exists)
        self.assertTrue(exist_service(test_tuple).exists)

        rospy.wait_for_service(self.__pop_service_name)
        pop_service = rospy.ServiceProxy(self.__pop_service_name, Pop)
        pop_response = pop_service(test_tuple)
        self.assertTrue(pop_response.exists)
        self.assertEqual(test_tuple, tuple(pop_response.removed))

        self.assertFalse(exist_service(test_tuple).exists)

    def test_placeholder(self):
        test_tuple = (self.__message_prefix, 'test_placeholder', '0', '0')
        pattern = (self.__message_prefix, 'test_placeholder', '*', '*')

        self.add_tuple(test_tuple)
        rospy.wait_for_service(self.__peek_service_name)
        peek_service = rospy.ServiceProxy(self.__peek_service_name, Peek)
        peek_response = peek_service(pattern)
        self.assertTrue(peek_response.exists)
        self.assertEqual(test_tuple, tuple(peek_response.example))

    def __wait_for_tuple(self, wait_for_it):
        """
        waits, until the requested tuple is contained in knowledge_base
        :param wait_for_it: tuple
        """
        rospy.wait_for_service(self.__exists_service_name)
        exist_service = rospy.ServiceProxy(self.__exists_service_name, Exists)
        while not exist_service(wait_for_it).exists:
            rospy.sleep(1)

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
        t1 = (self.__message_prefix, 'test_all', 'pos', '0', '0')
        self.add_tuple(t1)
        t2 = (self.__message_prefix, 'test_all', 'pos', '1', '0')
        self.add_tuple(t2)
        t3 = (self.__message_prefix, 'test_all', 'pos', '1', '-4')
        self.add_tuple(t3)

        self.__wait_for_tuple(t3)

        rospy.wait_for_service(self.__all_service_name)
        all_service = rospy.ServiceProxy(self.__all_service_name, All)
        all_response = all_service((self.__message_prefix, 'test_all', 'pos', '*', '*'))

        self.__check_content((self.__message_prefix, 'test_all', 'pos', '*', '*'), t1, t2, t3)

    def __check_content(self, pattern, *expected):
        rospy.wait_for_service(self.__all_service_name)
        all_service = rospy.ServiceProxy(self.__all_service_name, All)
        all_response = all_service(pattern)
        self.assertEqual(len(expected), len(all_response.found),
                         ' Expected: ' + str(expected) + ' but is ' + str(all_response))
        for fact in all_response.found:
            self.assertTrue(tuple(fact.content), ' Expected: ' + str(expected) + ' but is ' + str(all_response))

    def test_prevent_multiple_adding(self):
        """
        tests that no duplicates are contained in knowledge base
        """
        test_tuple = (self.__message_prefix, 'test_prevent_multiple_adding', '0', '0')
        self.add_tuple(test_tuple)
        self.add_tuple(test_tuple)

        rospy.wait_for_service(self.__all_service_name)
        all_service = rospy.ServiceProxy(self.__all_service_name, All)
        all_response = all_service(test_tuple)
        self.assertEqual(1, len(all_response.found))

    def __check_mocks_empty(self, update_mocks):
        for update_mock in update_mocks:
            self.assertIsNone(update_mock.add_msg,
                              'Following mock has an unexpected add message received: ' + update_mock.name)
            self.assertIsNone(update_mock.remove_msg,
                              'Following mock has an unexpected remove message received: ' + update_mock.name)
            self.assertIsNone(update_mock.update_msg,
                              'Following mock has an unexpected update message received: ' + update_mock.name)

    def test_update_non_existing(self):
        prefix = self.__message_prefix + '_test_update_non_existing'
        new_fact = (prefix, 'new', '1')
        not_influenced = (prefix, 'not_influenced', '1')

        self.add_tuple(not_influenced)

        informer_added = UpdateSubscriberMock(mock_name='added', knowledge_base_name=self.__knowledge_base_address,
                                              pattern=(prefix, 'new', '*'))
        informer_removed = UpdateSubscriberMock(mock_name='removed', knowledge_base_name=self.__knowledge_base_address,
                                                pattern=(prefix, 'unsuccess', '*'))
        informer_updated = UpdateSubscriberMock(mock_name='updated', knowledge_base_name=self.__knowledge_base_address,
                                                pattern=(prefix, '*', '*'))
        informer_not_influenced = UpdateSubscriberMock(mock_name='not_influended',
                                                       knowledge_base_name=self.__knowledge_base_address,
                                                       pattern=(prefix, 'not_influenced', '*'))
        all_informers = [informer_added, informer_removed, informer_updated, informer_not_influenced]

        rospy.wait_for_service(self.__update_service_name)
        update_service = rospy.ServiceProxy(self.__update_service_name, Update)
        not_success_response = update_service(
            UpdateRequest(fact=(self.__message_prefix, 'unsuccess', '1'), newFact=new_fact))
        rospy.sleep(0.1)
        self.assertFalse(not_success_response.successful, 'Old fact does not exist, but update was successfull')
        self.__check_mocks_empty(all_informers)
        self.__check_content((prefix, '*', '*'), not_influenced)

    def test_update_basic(self):
        prefix = self.__message_prefix + '_test_update_basic'
        old_fact = (prefix, 'old', '2')
        new_fact = (prefix, 'new', '1')
        new_fact2 = (prefix, 'seccond_target', '1')
        not_influenced = (prefix, 'not_influenced', '1')

        self.add_tuple(old_fact, not_influenced, new_fact2)

        informer_added = UpdateSubscriberMock(mock_name='added', knowledge_base_name=self.__knowledge_base_address,
                                              pattern=(prefix, 'new', '*'))
        informer_removed = UpdateSubscriberMock(mock_name='removed', knowledge_base_name=self.__knowledge_base_address,
                                                pattern=(prefix, 'old', '*'))
        informer_updated = UpdateSubscriberMock(mock_name='updated', knowledge_base_name=self.__knowledge_base_address,
                                                pattern=(prefix, '*', '*'))
        informer_seccond_target = UpdateSubscriberMock(mock_name='seccond',
                                                       knowledge_base_name=self.__knowledge_base_address,
                                                       pattern=(prefix, 'seccond_target', '*'))
        informer_not_influenced = UpdateSubscriberMock(mock_name='not_influended',
                                                       knowledge_base_name=self.__knowledge_base_address,
                                                       pattern=(prefix, 'not_influenced', '*'))
        all_informers = [informer_added, informer_removed, informer_updated, informer_seccond_target,
                         informer_not_influenced]

        rospy.wait_for_service(self.__update_service_name)
        update_service = rospy.ServiceProxy(self.__update_service_name, Update)

        update_response = update_service(fact=old_fact, newFact=new_fact)
        self.assertTrue(update_response.successful, 'First update failed')
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
        self.__check_content((prefix, '*', '*'), new_fact, new_fact2, not_influenced)

    def test_update_existing_target(self):
        prefix = self.__message_prefix + '_test_update_existing_target'
        old_fact = (prefix, 'old', '2')
        new_fact = (prefix, 'new', '1')
        not_influenced = (prefix, 'not_influenced', '1')
        self.add_tuple(old_fact, new_fact, not_influenced)

        informer_added = UpdateSubscriberMock(mock_name='added', knowledge_base_name=self.__knowledge_base_address,
                                              pattern=(prefix, 'new', '*'))
        informer_removed = UpdateSubscriberMock(mock_name='removed', knowledge_base_name=self.__knowledge_base_address,
                                                pattern=(prefix, 'old', '*'))
        informer_updated = UpdateSubscriberMock(mock_name='updated', knowledge_base_name=self.__knowledge_base_address,
                                                pattern=(prefix, '*', '*'))
        informer_seccond_target = UpdateSubscriberMock(mock_name='seccond',
                                                       knowledge_base_name=self.__knowledge_base_address,
                                                       pattern=(prefix, 'seccond_target', '*'))
        informer_not_influenced = UpdateSubscriberMock(mock_name='not_influended',
                                                       knowledge_base_name=self.__knowledge_base_address,
                                                       pattern=(prefix, 'not_influenced', '*'))
        all_informers = [informer_added, informer_removed, informer_updated, informer_seccond_target,
                         informer_not_influenced]

        rospy.wait_for_service(self.__update_service_name)
        update_service = rospy.ServiceProxy(self.__update_service_name, Update)
        update_response = update_service(fact=old_fact, newFact=new_fact)
        self.assertTrue(update_response.successful, 'Seccond update failed')
        rospy.sleep(0.1)

        self.assertIsNotNone(informer_removed.remove_msg, 'remove message was not received')
        self.assertEqual(old_fact, tuple(informer_removed.remove_msg.fact))
        informer_removed.remove_msg = None
        self.assertIsNotNone(informer_updated.remove_msg, 'remove message was not received')
        self.assertEqual(old_fact, tuple(informer_updated.remove_msg.fact))
        informer_updated.remove_msg = None

        self.__check_mocks_empty(all_informers)
        self.__check_content((prefix, '*', '*'), new_fact, not_influenced)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'knowledge_base_manager_test_node', TupleSpaceTestSuite)
