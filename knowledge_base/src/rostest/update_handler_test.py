#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: rieger, hrabia
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

        self.__knowledge_base_address = KnowledgeBase.DEFAULT_NAME + "UpdateHandlerTestSuite"
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
        node = roslaunch.core.Node(package=package, node_type=executable, name=self.__knowledge_base_address,
                                   output='screen')
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

        cache = KnowledgeBaseFactCache(pattern=test_tuple, knowledge_base_name=self.__knowledge_base_address)
        self.assertFalse(cache.does_fact_exists(), 'Tuple is not added, but is indicated as present')

        self.__client.push(test_tuple)
        rospy.sleep(0.1)

        self.assertTrue(cache.does_fact_exists(), 'Tuple was added, but is not indicated as present')

    def test_remove(self):
        test_tuple = (self.__message_prefix, 'test_remove', '0', '0')

        cache = KnowledgeBaseFactCache(pattern=test_tuple, knowledge_base_name=self.__knowledge_base_address)
        self.assertFalse(cache.does_fact_exists(), 'Tuple is not added, but is indicated as present')

        self.__client.push(test_tuple)
        rospy.sleep(0.1)

        self.assertTrue(cache.does_fact_exists(), 'Tuple was added, but is not indicated as present')

        self.__client.pop(test_tuple)

        rospy.sleep(0.1)
        self.assertFalse(cache.does_fact_exists(), 'Tuple was removed, but is still indicated as present')

    def test_multiple_caches_for_same_pattern(self):
        test_tuple = (self.__message_prefix, 'test_multiple_caches_for_same_pattern', '0', '0')

        cache1 = KnowledgeBaseFactCache(pattern=test_tuple, knowledge_base_name=self.__knowledge_base_address)
        cache2 = KnowledgeBaseFactCache(pattern=test_tuple, knowledge_base_name=self.__knowledge_base_address)

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
        pattern = (prefix, 'updated', '*', '1')
        cache = KnowledgeBaseFactCache(pattern=pattern, knowledge_base_name=self.__knowledge_base_address)

        rospy.sleep(0.1)

        current = cache.get_all_matching_facts()

        self.assertEqual(2, len(current))
        self.assertTrue(first_fact in current, 'fact not found')
        self.assertTrue(second_fact in current, 'fact not found')

    def test_update_existing(self):
        """
        Test updating a fact that is already stored.
        """
        prefix = self.__message_prefix + '_test_update_existing'

        updated_old = (prefix, 'updated', '1')

        self.assertTrue(self.__client.push(updated_old))

        rospy.sleep(0.1)

        cache = KnowledgeBaseFactCache(pattern=(prefix, '*', '*'), knowledge_base_name=self.__knowledge_base_address)
        rospy.sleep(0.1)
        updated_new = (prefix, 'updated', '1')
        self.assertTrue(self.__client.update((prefix, '*', '*'), updated_new))

        rospy.sleep(0.5)  # don' t wait for updates because we will not get one, as nothing changed

        current = cache.get_all_matching_facts()

        self.assertEqual(1, len(current))
        self.assertTrue(updated_new in current, 'Update corrupted')

    def test_update(self):
        prefix = self.__message_prefix + '_test_update'
        updated_old = (prefix, 'updated', '1')
        not_influenced = (prefix, 'not_influenced', '1')
        self.__client.push(updated_old)
        self.__client.push(not_influenced)

        cache = KnowledgeBaseFactCache(pattern=(prefix, '*', '*'), knowledge_base_name=self.__knowledge_base_address)

        update_stamp = cache.update_time

        updated_new = (prefix, 'updated', '0')

        self.__client.update(updated_old, updated_new)
        while update_stamp == cache.update_time:
            rospy.sleep(0.5)

        current = cache.get_all_matching_facts()
        self.assertEqual(2, len(current))
        self.assertTrue(updated_new in current, 'Update not noticed')
        self.assertTrue(not_influenced in current, 'NotInfluenced was influenced')

    def test_update_non_existing(self):
        prefix = self.__message_prefix + 'test_update_non_existing'
        updated_not_existing = (prefix, 'updated', '1')

        cache = KnowledgeBaseFactCache(pattern=(prefix, '*', '*'), knowledge_base_name=self.__knowledge_base_address)

        update_stamp = cache.update_time

        updated_new = (prefix, 'updated', '0')

        self.__client.update(updated_not_existing, updated_new)
        while update_stamp == cache.update_time:
            rospy.sleep(0.5)

        current = cache.get_all_matching_facts()
        self.assertEqual(1, len(current))
        self.assertTrue(updated_new in current, 'Update not noticed')

    def test_multiple_updates(self):
        prefix = self.__message_prefix + '_test_multiple_updates'
        updated_old_1 = (prefix, 'fact_1', '1')
        updated_old_2 = (prefix, 'fact_2', '2')
        self.__client.push(updated_old_1)
        self.__client.push(updated_old_2)

        cache = KnowledgeBaseFactCache(pattern=(prefix, '*', '*'), knowledge_base_name=self.__knowledge_base_address)

        updated_new_1 = (prefix, 'fact_1', '1*')
        self.__client.update(updated_old_1, updated_new_1)

        updated_new_2 = (prefix, 'fact_2', '2*')

        update_stamp = cache.update_time
        self.__client.update(updated_old_2, updated_new_2)

        rospy.sleep(0.5)
        while update_stamp == cache.update_time:
            rospy.sleep(0.5)

        current = cache.get_all_matching_facts()

        self.assertEqual(2, len(current))
        self.assertTrue(updated_new_1 in current, 'Update of updated_new_1 not noticed')
        self.assertTrue(updated_new_2 in current, 'Update of updated_new_2 not noticed')

    def test_update_with_partly_matching_patterns(self):
        """
        test an update that replaces several existing facts with a single new one or adds a new fact as a result of an
        update
        """
        prefix = self.__message_prefix + 'test_update_reduction'
        updated_old_1 = (prefix, 'fact_1', '1')
        updated_old_2 = (prefix, 'fact_2', '1')
        self.__client.push(updated_old_1)
        self.__client.push(updated_old_2)

        cache_all = KnowledgeBaseFactCache(pattern=(prefix, '*', '*'), knowledge_base_name=self.__knowledge_base_address)

        cache_specific_1 = KnowledgeBaseFactCache(pattern=(prefix, 'fact_1', '*'),
                                                  knowledge_base_name=self.__knowledge_base_address)

        cache_specific_2 = KnowledgeBaseFactCache(pattern=(prefix, 'fact_2', '*'),
                                                  knowledge_base_name=self.__knowledge_base_address)

        cache_specific_3 = KnowledgeBaseFactCache(pattern=(prefix, 'fact_3', '*'),
                                                  knowledge_base_name=self.__knowledge_base_address)

        current_all = cache_all.get_all_matching_facts()
        current_specific_1 = cache_specific_1.get_all_matching_facts()
        current_specific_2 = cache_specific_2.get_all_matching_facts()
        current_specific_3 = cache_specific_3.get_all_matching_facts()

        self.assertEqual(2, len(current_all))
        self.assertEqual(1, len(current_specific_1))
        self.assertEqual(1, len(current_specific_2))
        self.assertEqual(0, len(current_specific_3))

        updated_new = (prefix, 'fact_3', '1')
        update_stamp1 = cache_all.update_time
        update_stamp2 = cache_specific_1.update_time
        update_stamp3 = cache_specific_2.update_time
        update_stamp4 = cache_specific_3.update_time
        self.__client.update((prefix, '*', '1'), updated_new)

        # wait until the caches are updated
        rospy.sleep(0.5)
        while update_stamp1 == cache_all.update_time or \
                update_stamp2 == cache_specific_1.update_time or update_stamp3 == cache_specific_2.update_time \
                or update_stamp4 == cache_specific_3.update_time:
            rospy.sleep(0.1)

        current_all = cache_all.get_all_matching_facts()
        current_specific_1 = cache_specific_1.get_all_matching_facts()
        current_specific_2 = cache_specific_2.get_all_matching_facts()
        current_specific_3 = cache_specific_3.get_all_matching_facts()

        self.assertEqual(1, len(current_all))
        self.assertEqual(0, len(current_specific_1))
        self.assertEqual(0, len(current_specific_2))
        self.assertEqual(1, len(current_specific_3))
        self.assertTrue(updated_new in current_all, 'Update general cache of not noticed')
        self.assertTrue(updated_new not in current_specific_1, 'Delete update in specific cache 1 not noticed')
        self.assertTrue(updated_new not in current_specific_2, 'Delete update in specific cache 2 not noticed')
        self.assertTrue(updated_new in current_specific_3, 'Add update in specific cache 3 not noticed')

    def test_all_pattern_with_update(self):
        prefix = self.__message_prefix + '_test_multiple_updates'
        updated_old_1 = (prefix, 'fact_1', '1')
        self.__client.push(updated_old_1)

        all_pattern = ()
        cache = KnowledgeBaseFactCache(pattern=all_pattern, knowledge_base_name=self.__knowledge_base_address)
        update_stamp = cache.update_time

        updated_new_1 = (prefix, 'fact_1', '3')
        self.__client.update(updated_old_1, updated_new_1)

        while update_stamp == cache.update_time:
            rospy.sleep(0.1)

        current = cache.get_all_matching_facts()
        self.assertEqual(1, len(current))
        self.assertTrue(updated_new_1 in current, 'Update not noticed')

    def __check_content(self, fact_cache, *expected_facts):

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

        cache = KnowledgeBaseFactCache(pattern=(prefix, '*', '*'), knowledge_base_name=self.__knowledge_base_address)

        rospy.sleep(0.1)

        self.__client.push(updated_old_1)
        self.__client.push(updated_old_2)
        self.__client.push(not_influenced)

        rospy.sleep(0.1)

        self.__check_content(cache,updated_old_1,updated_old_2, not_influenced)

        new_fact = (prefix, 'updated', '3')

        self.__client.update((prefix, 'toUpdate', '*'), new_fact)

        rospy.sleep(0.1)

        self.__check_content(cache, not_influenced, new_fact)

    def test_mass_updates(self):
        """
        Here we test many updates in a short time period
        """

        prefix = self.__message_prefix + 'test_mass_updates'

        number_of_entries = 500

        # fill KB
        for i in range(number_of_entries):

            updated_old_1 = (prefix, 'fact', str(i))
            self.__client.push(updated_old_1)

        cache = KnowledgeBaseFactCache(pattern=(prefix, 'fact_new', '*'), knowledge_base_name=self.__knowledge_base_address)
        current = cache.get_all_matching_facts()
        self.assertEqual(0, len(current))

        rospy.sleep(0.5)

        for i in range(number_of_entries):
            updated_old_1 = (prefix, 'fact', str(i))
            updated_new_1 = (prefix, 'fact_new', str(i))
            last_fact = updated_new_1
            self.__client.update(updated_old_1, updated_new_1)

        rospy.sleep(0.5)
        while last_fact != cache.last_updated_fact:
            rospy.sleep(0.5)

        current = cache.get_all_matching_facts()

        # test if we received all tuples
        for i in range(number_of_entries):
            updated_new_1 = (prefix, 'fact_new', str(i))
            self.assertTrue(updated_new_1 in current, 'Update of "' + str(updated_new_1) + '" not noticed')

    def test_sub_fact_matching(self):
        prefix = self.__message_prefix + '_test_multiple_updates'
        fact_1 = (prefix, 'fact_1', '1', 'a')
        self.__client.push(fact_1)
        fact_2 = (prefix, 'fact_2', '2', 'a')
        self.__client.push(fact_2)
        fact_3 = (prefix, 'fact_3', '3', 'b')
        self.__client.push(fact_3)

        all_pattern = ()
        cache = KnowledgeBaseFactCache(pattern=all_pattern, knowledge_base_name=self.__knowledge_base_address)

        # initial check to see if we have a correct setup
        current = cache.get_all_matching_facts()
        self.assertEqual(3, len(current))

        sub_fact_pattern = (prefix, 'fact_2', '2', 'a')
        does_exist = cache.does_sub_fact_exist(pattern=sub_fact_pattern)

        self.assertTrue(does_exist)

        first_sub_fact = cache.get_matching_sub_fact(pattern=sub_fact_pattern)

        self.assertEqual(first_sub_fact, fact_2)

        # test with sub placeholders
        sub_fact_pattern = (prefix, '*', '*', 'a')
        does_exist = cache.does_sub_fact_exist(pattern=sub_fact_pattern)

        self.assertTrue(does_exist)

        first_sub_fact = cache.get_matching_sub_fact(pattern=sub_fact_pattern)

        self.assertEqual(first_sub_fact, fact_1)

        all_sub_facts = cache.get_all_matching_sub_facts(pattern=sub_fact_pattern)
        self.assertEqual(2, len(all_sub_facts))
        self.assertTrue(fact_1 in all_sub_facts, "fact_1 not found in sub facts")
        self.assertTrue(fact_2 in all_sub_facts, "fact_2 not found in sub facts")

        # invalid pattern
        sub_fact_pattern = (prefix, 'fact_unknown', '2', 'a')
        does_exist = cache.does_sub_fact_exist(pattern=sub_fact_pattern)

        self.assertFalse(does_exist)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'update_handler_test_node', UpdateHandlerTestSuite)
