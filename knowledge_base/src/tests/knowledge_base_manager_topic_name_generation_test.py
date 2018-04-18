#! /usr/bin/env python2
import unittest

import rostest
from knowledge_base.knowledge_base_manager import KnowledgeBase


class KnowledgeBaseManagerTopicNameGenerationTestSuite(unittest.TestCase):
    """
    Tests the topic name generation for fact updates
    """

    def test_without_pattern(self):
        pattern = ('pos', '0', '0')
        topic_name = KnowledgeBase.generate_topic_name_for_pattern('/knowledgebase/FactUpdate/', pattern,
                                                                   include_pattern=False, counter=0)
        self.assertEqual(topic_name, '/knowledgebase/FactUpdate/Topic0')

    def test_simple(self):
        pattern = ('pos', '0', '0')
        topic_name = KnowledgeBase.generate_topic_name_for_pattern('/knowledgebase/FactUpdate/', pattern,
                                                                   include_pattern=True, counter=0)
        self.assertEqual(topic_name, '/knowledgebase/FactUpdate/Topic0_pos_0_0')

    def test_placeholder(self):
        """
        tests name generation with placeholder
        """
        pattern = ('pos', 'str', str)
        topic_name = KnowledgeBase.generate_topic_name_for_pattern('/knowledgebase/FactUpdate/', pattern,
                                                                   include_pattern=True, counter=0)
        self.assertEqual(topic_name, '/knowledgebase/FactUpdate/Topic0_pos_str_x')

    def test_filter_illegal_chars(self):
        """
        Tests filtering illegal chars
        """
        pattern = ('test_pos', '0.0', '0}')
        topic_name = KnowledgeBase.generate_topic_name_for_pattern('/knowledgebase/FactUpdate/', pattern,
                                                                   include_pattern=True, counter=0)
        self.assertEqual('/knowledgebase/FactUpdate/Topic0_testpos_00_0', topic_name)

    def test_all_allowed_chars(self):
        """
        Tests that no legal char is filtered
        """
        pattern = ('abcdefghijklmnopqrstuvwxyz', 'ABCDEFGHIJKLMNOPQRSTUVWXYZ', '0123456789')
        topic_name = KnowledgeBase.generate_topic_name_for_pattern('/knowledgebase/FactUpdate/', pattern,
                                                                   include_pattern=True, counter=0)
        self.assertEqual('/knowledgebase/FactUpdate/Topic0_abcdefghijklmnopqrstuvwxyz_ABCDEFGHIJKLMNOPQRSTUVWXYZ_0123456789',
                         topic_name)


if __name__ == '__main__':
    unittest.main()
