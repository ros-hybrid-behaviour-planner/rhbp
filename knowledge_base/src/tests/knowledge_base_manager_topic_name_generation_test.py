#! /usr/bin/env python2
import unittest

import rostest
from knowledge_base.knowledge_base_manager import KnowledgeBase


PKG = 'knowledge_base'


class KnowledgeBaseManagerTopicNameGenerationTestSuite(unittest.TestCase):

    def test_simple(self):
        pattern = ('pos','0','0')
        topic_name = KnowledgeBase.generate_topic_name_for_pattern('/knowledgebase/FactUpdate/',pattern)
        self.assertEqual(topic_name,'/knowledgebase/FactUpdate/pos_0_0')

    def test_placeholder(self):
        pattern = ('pos','str',str)
        topic_name = KnowledgeBase.generate_topic_name_for_pattern('/knowledgebase/FactUpdate/',pattern)
        self.assertEqual(topic_name,'/knowledgebase/FactUpdate/pos_str_x')

    def test_filter_illegal_chars(self):
        pattern = ('test_pos','0.0','0}')
        topic_name = KnowledgeBase.generate_topic_name_for_pattern('/knowledgebase/FactUpdate/',pattern)
        self.assertEqual('/knowledgebase/FactUpdate/testpos_00_0',topic_name)

    def test_all_allowed_chars(self):
        pattern = ('abcdefghijklmnopqrstuvwxyz','ABCDEFGHIJKLMNOPQRSTUVWXYZ','0123456789')
        topic_name = KnowledgeBase.generate_topic_name_for_pattern('/knowledgebase/FactUpdate/',pattern)
        self.assertEqual('/knowledgebase/FactUpdate/abcdefghijklmnopqrstuvwxyz_ABCDEFGHIJKLMNOPQRSTUVWXYZ_0123456789',topic_name)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'knowledge_base_manager_topic_name_generation_test', KnowledgeBaseManagerTopicNameGenerationTestSuite)