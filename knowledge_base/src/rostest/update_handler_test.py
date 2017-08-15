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
from knowledge_base.update_handler import KnowledgeBaseFactCache

PKG = 'knowledge_base'

"""
System test for knowledge base fact cache.
"""


class UpdateHandlerTestSuite(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(UpdateHandlerTestSuite, self).__init__(*args, **kwargs)

    def setUp(self):

        self.__knowledge_base_address = KnowledgeBase.DEFAULT_NAME
        # prevent influence of previous tests
        self.__message_prefix = 'UpdateHandlerTestSuite' + str(time.time())

        # start KnowledgeBase
        self.start_kb_node()

        rospy.init_node('UpdateHandlerTestSuite', log_level=rospy.DEBUG)

        self.__client = KnowledgeBaseClient(self.__knowledge_base_address)

    def start_kb_node(self):
        """
        start the knowledge base node
        """
        package = 'knowledge_base'
        executable = 'knowledge_base_node.py'
        node = roslaunch.core.Node(package=package, node_type=executable, name=self.__knowledge_base_address)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        self._kb_process = launch.launch(node)

    def tearDown(self):
        self._kb_process.stop()

    def test_simple_adding(self):
        """
        Tests basic case of adding a tuple
        """
        test_tuple = (self.__message_prefix, 'test_simple_adding', '0', '0')

        cache = KnowledgeBaseFactCache(pattern=test_tuple)
        self.assertFalse(cache.does_fact_exists(), 'Tuple is not added, but is indicated as present')

        self.__client.push(test_tuple)
        rospy.sleep(0.1)

        self.assertTrue(cache.does_fact_exists(), 'Tuple was added, but is not indicated as present')

    def test_remove(self):
        test_tuple = (self.__message_prefix, 'test_remove', '0', '0')

        cache = KnowledgeBaseFactCache(pattern=test_tuple)
        self.assertFalse(cache.does_fact_exists(), 'Tuple is not added, but is indicated as present')

        self.__client.push(test_tuple)
        rospy.sleep(0.1)

        self.assertTrue(cache.does_fact_exists(), 'Tuple was added, but is not indicated as present')

        self.__client.pop(test_tuple)

        rospy.sleep(0.1)
        self.assertFalse(cache.does_fact_exists(), 'Tuple was removed, but is still indicated as present')

    def test_multiple_caches_for_same_pattern(self):
        test_tuple = (self.__message_prefix, 'test_multiple_caches_for_same_pattern', '0', '0')

        cache1 = KnowledgeBaseFactCache(pattern=test_tuple)
        cache2 = KnowledgeBaseFactCache(pattern=test_tuple)

        self.assertFalse(cache1.does_fact_exists(), 'Tuple is not added, but is indicated as present')
        self.assertFalse(cache2.does_fact_exists(), 'Tuple is not added, but is indicated as present')

        self.__client.push(test_tuple)
        rospy.sleep(0.1)

        self.assertTrue(cache1.does_fact_exists(), 'Tuple was added, but is not indicated as present')
        self.assertTrue(cache2.does_fact_exists(), 'Tuple was added, but is not indicated as present')

    def test_middle_placeholder(self):
        """
        Test updating an existing fact 
        """
        prefix = self.__message_prefix + '_test_middle_placeholder'

        first_fact = (prefix, 'updated', 'a', '1')

        second_fact = (prefix, 'updated', 'b', '1')

        self.__client.push(first_fact)
        self.__client.push(second_fact)

        cache = KnowledgeBaseFactCache(pattern=(prefix, 'updated', '*', '1'))

        rospy.sleep(0.1)

        current = cache.get_all_matching_facts()

        self.assertEqual(2, len(current))
        self.assertTrue(first_fact in current, 'fact not found')
        self.assertTrue(second_fact in current, 'fact not found')

    def test_update_existing(self):
        """
        Test updating an existing fact 
        """
        prefix = self.__message_prefix + '_test_update_empty'

        updated_old = (prefix, 'updated', '1')

        self.__client.push(updated_old)

        cache = KnowledgeBaseFactCache(pattern=(prefix, '*', '*'))

        updated_new = (prefix, 'updated', '1')

        self.__client.update((prefix, '*','*'), updated_new)
        rospy.sleep(0.1)

        current = cache.get_all_matching_facts()

        self.assertEqual(1, len(current))
        self.assertTrue(updated_new in current, 'Update not noticed')

    def test_update(self):
        prefix = self.__message_prefix + '_test_update'
        updated_old = (prefix, 'updated', '1')
        not_influenced = (prefix, 'not_influenced', '1')
        self.__client.push(updated_old)
        self.__client.push(not_influenced)

        cache = KnowledgeBaseFactCache(pattern=(prefix, '*', '*'))

        updated_new = (prefix, 'updated', '0')

        self.__client.update(updated_old, updated_new)
        rospy.sleep(0.1)

        current = cache.get_all_matching_facts()
        self.assertEqual(2, len(current))
        self.assertTrue(updated_new in current, 'Update not noticed')
        self.assertTrue(not_influenced in current, 'NotInfluenced was influenced')

    def test_multiple_updates(self):
        prefix = self.__message_prefix + '_test_multiple_updates'
        updated_old_1 = (prefix, 'fact_1', '1')
        updated_old_2 = (prefix, 'fact_2', '2')
        self.__client.push(updated_old_1)
        self.__client.push(updated_old_2)

        cache = KnowledgeBaseFactCache(pattern=(prefix, '*', '*'))

        updated_new_1 = (prefix, 'fact_1', '3')
        self.__client.update(updated_old_1, updated_new_1)

        updated_new_2 = (prefix, 'fact_2', '4')
        self.__client.update(updated_old_2, updated_new_2)
        rospy.sleep(0.1)

        current = cache.get_all_matching_facts()
        self.assertEqual(2, len(current))
        self.assertTrue(updated_new_1 in current, 'Update not noticed')
        self.assertTrue(updated_new_2 in current, 'Update not noticed')

    def __check_content(self,fact_cache, *expected_facts):

        content = fact_cache.get_all_matching_facts()
        error_message = 'Excepted "{0}", but is {1}'.format(str(expected_facts), str(content))
        self.assertEqual(len(content), len(expected_facts),error_message)
        for expected_fact in expected_facts:
            self.assertTrue(expected_fact in content, error_message)
        for existing_fact in content:
            self.assertTrue(existing_fact in expected_facts, error_message)

    def test_update_replacing_several_facts(self):

        prefix = self.__message_prefix + 'test_update_replacing_several_facts'
        updated_old_1 = (prefix, 'toUpdate', '1')
        updated_old_2 = (prefix, 'toUpdate', '2')
        not_influenced = (prefix, 'not_influenced', '1')

        cache = KnowledgeBaseFactCache(pattern=(prefix, '*', '*'))

        rospy.sleep(0.1)

        self.__client.push(updated_old_1)
        self.__client.push(updated_old_2)
        self.__client.push(not_influenced)

        rospy.sleep(0.1)

        self.__check_content(cache,updated_old_1,updated_old_2, not_influenced)

        new_fact = (prefix, 'updated', '3')

        self.__client.update((prefix,'toUpdate','*'),new_fact)

        rospy.sleep(0.1)

        self.__check_content(cache, not_influenced, new_fact)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'update_handler_test_node', UpdateHandlerTestSuite)
