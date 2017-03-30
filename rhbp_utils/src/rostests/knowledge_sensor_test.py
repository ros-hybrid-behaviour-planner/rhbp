#!/usr/bin/env python
"""
Tests the knowledge sensor.
Requires a running roscore and a knowledge_base

@author: rieger
"""
import time
import unittest

import rospy
import rostest
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from knowledge_base.knowledge_base_client import KnowledgeBaseClient

PKG = 'rhbp_core'

"""
Integration test for KnowledgeSensor.
Requires a running KnowledgeBase
"""


class TestKnowledgeBaseSensor(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestKnowledgeBaseSensor, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TestKnowledgeBaseSensor' + str(time.time())
        rospy.init_node('knowledge_sensor_test_node', log_level=rospy.DEBUG)
        self.__client = KnowledgeBaseClient()

    def test_basic(self):
        """
        Tests sensor output, if the fact is added at runtime (and did not exist before)
        """
        sensor = KnowledgeSensor(pattern=((self.__message_prefix, 'test_basic', 'pos', '*', '*')))
        sensor.sync()
        self.assertFalse(sensor.value)

        self.__client.push((self.__message_prefix, 'test_basic', 'pos', '42', '0'))
        rospy.sleep(0.1)

        sensor.sync()
        self.assertTrue(sensor.value)

    def test_remove(self):
        """
        Tests sensor output , if the fact is removed
        """
        test_tuple = (self.__message_prefix, 'test_remove', 'pos', '42', '0')
        self.__client.push(test_tuple)
        rospy.sleep(0.1)

        sensor = KnowledgeSensor(pattern=(self.__message_prefix, 'test_remove', 'pos', '*', '*'))
        sensor.sync()
        self.assertTrue(sensor.value)

        rospy.sleep(0.1)

        self.__client.pop(test_tuple)

        rospy.sleep(0.1)

        sensor.sync()

        self.assertFalse(sensor.value)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'knowledge_sensor_test_node', TestKnowledgeBaseSensor)
