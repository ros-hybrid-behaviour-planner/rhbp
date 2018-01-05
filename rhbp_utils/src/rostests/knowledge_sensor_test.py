#!/usr/bin/env python
"""
Tests the knowledge sensor.
Requires a running roscore and a knowledge_base

@author: rieger, hrabia
"""
import time
import unittest

import rospy
import rostest
import roslaunch
from rhbp_utils.knowledge_sensors import KnowledgeSensor, KnowledgeFactNumberSensor
from knowledge_base.knowledge_base_client import KnowledgeBaseClient, KnowledgeBase

PKG = 'rhbp_core'

"""
Integration test for KnowledgeSensor.
Requires a running KnowledgeBase
"""


class TestKnowledgeBaseSensor(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestKnowledgeBaseSensor, self).__init__(*args, **kwargs)

        self.__knowledge_base_address = KnowledgeBase.DEFAULT_NAME

        # prevent influence of previous tests
        self.__message_prefix = 'TestKnowledgeBaseSensor' + str(time.time())

        self.start_kb_node()

        rospy.init_node('knowledge_sensor_test_node', log_level=rospy.DEBUG)
        self.__client = KnowledgeBaseClient()

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

    def test_basic(self):
        """
        Tests sensor output, if the fact is added at runtime (and did not exist before)
        """
        sensor = KnowledgeSensor(pattern=(self.__message_prefix, 'test_basic', 'pos', '*', '*'))
        sensor.sync()
        self.assertFalse(sensor.value)

        self.__client.push((self.__message_prefix, 'test_basic', 'pos', '42', '0'))
        rospy.sleep(1.0)  # long sleep times are really required here to make sure that all ROS msgs are transferred

        sensor.sync()
        self.assertTrue(sensor.value)

    def test_remove(self):
        """
        Tests sensor output , if the fact is removed
        """
        test_tuple = (self.__message_prefix, 'test_remove', 'pos', '42', '0')
        self.__client.push(test_tuple)

        sensor = KnowledgeSensor(pattern=(self.__message_prefix, 'test_remove', 'pos', '*', '*'))
        rospy.sleep(1.0)  # long sleep times are really required here to make sure that all ROS msgs are transferred
        sensor.sync()
        self.assertTrue(sensor.value)

        rospy.sleep(1.0)  # long sleep times are really required here to make sure that all ROS msgs are transferred

        removed_facts = self.__client.pop(test_tuple)

        self.assertIsNotNone(removed_facts, "No fact removed")

        rospy.sleep(1.0)  # long sleep times are really required here to make sure that all ROS msgs are transferred

        sensor.sync()

        self.assertFalse(sensor.value)

    def test_knowledge_fact_int_sensor(self):
        """
        Test KnowledgeFactIntSensor
        """
        initial_value = 1337
        sensor_pattern = (self.__message_prefix, 'test_knowledge_fact_int_sensor', 'number', '*')
        sensor = KnowledgeFactNumberSensor(pattern=sensor_pattern, initial_value=initial_value)
        sensor.sync()
        self.assertEquals(sensor.value, initial_value)

        test_value = 42

        # regular operation
        self.__client.push((self.__message_prefix, 'test_knowledge_fact_int_sensor', 'number', str(test_value)))
        rospy.sleep(1.0)  # long sleep times are really required here to make sure that all ROS msgs are transferred

        sensor.sync()
        self.assertEquals(sensor.value, test_value)

        # illegal operation with non integer value
        self.__client.update(pattern=sensor_pattern, new=(self.__message_prefix, 'test_knowledge_fact_int_sensor',
                                                          'number', "NO_NUMBER"))
        rospy.sleep(1.0)  # long sleep times are really required here to make sure that all ROS msgs are transferred

        sensor.sync()
        rospy.loginfo(sensor.value)
        self.assertEquals(sensor.value, initial_value)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'knowledge_sensor_test_node', TestKnowledgeBaseSensor)
