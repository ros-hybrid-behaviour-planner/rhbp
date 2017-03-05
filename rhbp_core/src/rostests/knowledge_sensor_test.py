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
from behaviour_components.sensors import KnowledgeSensor
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.msg import Push
from knowledge_base.srv import Pop

PKG = 'rhbp_core'


class TestKnowledgeBaseSensor(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestKnowledgeBaseSensor, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TestKnowledgeBaseSensor' + str(time.time())
        rospy.init_node('knowledge_sensor_test_node', log_level=rospy.DEBUG)
        self.__pop_service_name = KnowledgeBase.DEFAULT_NAME + KnowledgeBase.POP_SERVICE_NAME_POSTFIX

    @staticmethod
    def add_tuple(to_add):
        """
        Adds the given fact to knowledge base
        :param to_add: array of strings
        """
        pub = rospy.Publisher(KnowledgeBase.DEFAULT_NAME + KnowledgeBase.PUSH_TOPIC_NAME_POSTFIX, Push, queue_size=10)
        rospy.sleep(1)
        pub.publish(to_add)

    def test_basic(self):
        """
        Tests sensor output, if the fact is added at runtime (and did not exist before)
        """
        sensor = KnowledgeSensor(pattern=((self.__message_prefix, 'test_basic', 'pos', '*', '*')))
        sensor.sync()
        self.assertFalse(sensor.value)

        TestKnowledgeBaseSensor.add_tuple((self.__message_prefix, 'test_basic', 'pos', '42', '0'))
        rospy.sleep(1)

        sensor.sync()
        self.assertTrue(sensor.value)

    def test_remove(self):
        """
        Tests sensor output , if the fact is removed
        """
        test_tuple = (self.__message_prefix, 'test_remove', 'pos', '42', '0')
        TestKnowledgeBaseSensor.add_tuple(test_tuple)
        rospy.sleep(1)

        sensor = KnowledgeSensor(pattern=(self.__message_prefix, 'test_remove', 'pos', '*', '*'))
        sensor.sync()
        self.assertTrue(sensor.value)

        rospy.sleep(1)

        rospy.wait_for_service(self.__pop_service_name)
        pop_service = rospy.ServiceProxy(self.__pop_service_name, Pop)

        pop_service(test_tuple)

        rospy.sleep(1)

        sensor.sync()

        self.assertFalse(sensor.value)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'knowledge_sensor_test_node', TestKnowledgeBaseSensor)
