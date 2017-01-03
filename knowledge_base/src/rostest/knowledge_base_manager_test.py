#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: rieger
'''
import time
import unittest

import rospy
import rostest
from knowledge_base.msg import Push
from knowledge_base.srv import Exists, Peek, Pop, All
from knowledge_base.knowledge_base_manager import KnowledgeBase

PKG = 'knowledge_base'

"""
System test for knowledge base. Assumes, that a rosmaster and the knowledge base is running
"""


class TupleSpaceTestSuite(unittest.TestCase):


    def __init__(self, *args, **kwargs):
        super(TupleSpaceTestSuite, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TupleSpaceTestSuite' + str(time.time())
        self.__exists_service_name = KnowledgeBase.DEFAULT_NAME +KnowledgeBase.EXISTS_SERVICE_NAME_POSTFIX
        self.__peek_service_name =  KnowledgeBase.DEFAULT_NAME +KnowledgeBase.PEEK_SERVICE_NAME_POSTFIX
        self.__all_service_name =  KnowledgeBase.DEFAULT_NAME +KnowledgeBase.ALL_SERVICE_NAME_POSTFIX
        self.__pop_service_name =  KnowledgeBase.DEFAULT_NAME +KnowledgeBase.POP_SERVICE_NAME_POSTFIX

    @staticmethod
    def add_tuple(to_add):
        pub = rospy.Publisher(KnowledgeBase.DEFAULT_NAME+KnowledgeBase.PUSH_TOPIC_NAME_POSTFIX, Push, queue_size=10)
        rospy.sleep(1)
        pub.publish(to_add)

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

        rospy.wait_for_service( self.__pop_service_name)
        pop_service = rospy.ServiceProxy( self.__pop_service_name, Pop)
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

    def __wait_for_tuple(self,wait_for_it):
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

        self.assertEqual(3, len(all_response.found))
        self.assertTrue(TupleSpaceTestSuite.__is_tuple_in_facts(t1, all_response.found))
        self.assertTrue(TupleSpaceTestSuite.__is_tuple_in_facts(t2, all_response.found))
        self.assertTrue(TupleSpaceTestSuite.__is_tuple_in_facts(t3, all_response.found))

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


if __name__ == '__main__':
    rospy.init_node('TupleSpaceTestSuite', log_level=rospy.DEBUG)
    rostest.rosrun(PKG, 'knowledge_base_manager_test_node', TupleSpaceTestSuite)
