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
from knowledge_base.msg import Push
from knowledge_base.srv import Pop
from knowledge_base.update_handler import KnowledgeBaseFactCache

PKG = 'knowledge_base'

"""
System test for knowledge base fact cache. Assumes, that a rosmaster and the knowledge base is running
"""


class UpdateHandlerTestSuite(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(UpdateHandlerTestSuite, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'UpdateHandlerTestSuite' + str(time.time())
        self.__pop_service_name = KnowledgeBase.DEFAULT_NAME + KnowledgeBase.POP_SERVICE_NAME_POSTFIX

    @staticmethod
    def __add_tuple(to_add):
        pub = rospy.Publisher(KnowledgeBase.DEFAULT_NAME + KnowledgeBase.PUSH_TOPIC_NAME_POSTFIX, Push, queue_size=10)
        rospy.sleep(1)
        pub.publish(to_add)

    def test_simple_adding(self):
        test_tuple = (self.__message_prefix, 'test_simple_adding', '0', '0')

        cache = KnowledgeBaseFactCache(pattern=test_tuple)
        self.assertFalse(cache.does_fact_exists(), 'Tuple is not added, but is indicated as present')

        UpdateHandlerTestSuite.__add_tuple(test_tuple)
        rospy.sleep(1)

        self.assertTrue(cache.does_fact_exists(), 'Tuple was added, but is not indicated as present')

    def test_remove(self):
        test_tuple = (self.__message_prefix, 'test_remove', '0', '0')

        cache = KnowledgeBaseFactCache(pattern=test_tuple)
        self.assertFalse(cache.does_fact_exists(), 'Tuple is not added, but is indicated as present')

        rospy.sleep(1)
        UpdateHandlerTestSuite.__add_tuple(test_tuple)
        rospy.sleep(1)

        self.assertTrue(cache.does_fact_exists(), 'Tuple was added, but is not indicated as present')

        rospy.wait_for_service(self.__pop_service_name)
        pop_service = rospy.ServiceProxy(self.__pop_service_name, Pop)
        pop_service(test_tuple)

        rospy.sleep(1)
        self.assertFalse(cache.does_fact_exists(), 'Tuple was removed, but is still indicated as present')

    def test_multiple_caches_for_same_pattern(self):
        test_tuple = (self.__message_prefix, 'test_multiple_caches_for_same_pattern', '0', '0')

        cache1 = KnowledgeBaseFactCache(pattern=test_tuple)
        cache2 = KnowledgeBaseFactCache(pattern=test_tuple)
        rospy.sleep(1)

        self.assertFalse(cache1.does_fact_exists(), 'Tuple is not added, but is indicated as present')
        self.assertFalse(cache2.does_fact_exists(), 'Tuple is not added, but is indicated as present')

        UpdateHandlerTestSuite.__add_tuple(test_tuple)
        rospy.sleep(1)

        self.assertTrue(cache1.does_fact_exists(), 'Tuple was added, but is not indicated as present')
        self.assertTrue(cache2.does_fact_exists(), 'Tuple was added, but is not indicated as present')


if __name__ == '__main__':
    rospy.init_node('UpdateHandlerTestSuite', log_level=rospy.DEBUG)
    rostest.rosrun(PKG, 'update_handler_test_node', UpdateHandlerTestSuite)
