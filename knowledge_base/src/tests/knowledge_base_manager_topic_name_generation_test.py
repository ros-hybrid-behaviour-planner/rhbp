#! /usr/bin/env python2
import unittest

from knowledge_base.knowledge_base_manager import KnowledgeBase


class KnowledgeBaseManagerTopicNameGenerationTestSuite(unittest.TestCase):

    def test_simple(self):
        pattern = ('pos','0','0')
        topic_name = KnowledgeBase.generate_topic_name_for_pattern('/knowledgebase/FactUpdate/',pattern)
        self.assertEqual(topic_name,'/knowledgebase/FactUpdate/pos_0_0')

    def test_placeholder(self):
        pattern = ('pos','str',str)
        topic_name = KnowledgeBase.generate_topic_name_for_pattern('/knowledgebase/FactUpdate/',pattern)
        self.assertEqual(topic_name,'/knowledgebase/FactUpdate/pos_str_~')

if __name__ == '__main__':
    unittest.main()