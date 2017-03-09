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
from knowledge_base.srv import Pop, Update, UpdateRequest
from knowledge_base.update_handler import KnowledgeBaseFactCache

PKG = 'knowledge_base'

"""
System test for knowledge base fact cache. Assumes, that a rosmaster and the knowledge base is running
"""


class UpdateHandlerTestSuite(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(UpdateHandlerTestSuite, self).__init__(*args, **kwargs)
        rospy.init_node('UpdateHandlerTestSuite', log_level=rospy.DEBUG)
        # prevent influence of previous tests
        self.__message_prefix = 'UpdateHandlerTestSuite' + str(time.time())
        self.__pop_service_name = KnowledgeBase.DEFAULT_NAME + KnowledgeBase.POP_SERVICE_NAME_POSTFIX
        self.__update_service_name = KnowledgeBase.DEFAULT_NAME + KnowledgeBase.UPDATE_SERVICE_NAME_POSTFIX

    @staticmethod
    def __add_tuple(*to_add):
        pub = rospy.Publisher(KnowledgeBase.DEFAULT_NAME + KnowledgeBase.PUSH_TOPIC_NAME_POSTFIX, Push, queue_size=10)
        rospy.sleep(0.1)
        for fact in to_add:
            pub.publish(fact)

    def test_simple_adding(self):
        test_tuple = (self.__message_prefix, 'test_simple_adding', '0', '0')

        cache = KnowledgeBaseFactCache(pattern=test_tuple)
        self.assertFalse(cache.does_fact_exists(), 'Tuple is not added, but is indicated as present')

        UpdateHandlerTestSuite.__add_tuple(test_tuple)
        rospy.sleep(0.1)

        self.assertTrue(cache.does_fact_exists(), 'Tuple was added, but is not indicated as present')

    def test_remove(self):
        test_tuple = (self.__message_prefix, 'test_remove', '0', '0')

        cache = KnowledgeBaseFactCache(pattern=test_tuple)
        self.assertFalse(cache.does_fact_exists(), 'Tuple is not added, but is indicated as present')

        UpdateHandlerTestSuite.__add_tuple(test_tuple)
        rospy.sleep(0.1)

        self.assertTrue(cache.does_fact_exists(), 'Tuple was added, but is not indicated as present')

        rospy.wait_for_service(self.__pop_service_name)
        pop_service = rospy.ServiceProxy(self.__pop_service_name, Pop)
        pop_service(test_tuple)

        rospy.sleep(0.1)
        self.assertFalse(cache.does_fact_exists(), 'Tuple was removed, but is still indicated as present')

    def test_multiple_caches_for_same_pattern(self):
        test_tuple = (self.__message_prefix, 'test_multiple_caches_for_same_pattern', '0', '0')

        cache1 = KnowledgeBaseFactCache(pattern=test_tuple)
        cache2 = KnowledgeBaseFactCache(pattern=test_tuple)

        self.assertFalse(cache1.does_fact_exists(), 'Tuple is not added, but is indicated as present')
        self.assertFalse(cache2.does_fact_exists(), 'Tuple is not added, but is indicated as present')

        UpdateHandlerTestSuite.__add_tuple(test_tuple)
        rospy.sleep(1.0)

        self.assertTrue(cache1.does_fact_exists(), 'Tuple was added, but is not indicated as present')
        self.assertTrue(cache2.does_fact_exists(), 'Tuple was added, but is not indicated as present')

    def test_update(self):
        prefix = self.__message_prefix + '_test_update'
        updated_old = (prefix, 'updated', '1')
        not_influenced = (prefix, 'not_influenced', '1')
        UpdateHandlerTestSuite.__add_tuple(updated_old,not_influenced)

        cache = KnowledgeBaseFactCache(pattern=(prefix,'*','*'))

        updated_new = (prefix, 'updated', '0')

        rospy.wait_for_service(self.__update_service_name)
        update_service = rospy.ServiceProxy(self.__update_service_name, Update)
        update_service(UpdateRequest(fact=updated_old, newFact=updated_new))
        rospy.sleep(0.1)

        current = []
        for fact in cache.get_all_matching_facts():
            current.append(tuple(fact))
        self.assertEqual(2,len(current))
        self.assertTrue(updated_new in current)
        self.assertTrue(not_influenced in current)




if __name__ == '__main__':
    rostest.rosrun(PKG, 'update_handler_test_node', UpdateHandlerTestSuite)
