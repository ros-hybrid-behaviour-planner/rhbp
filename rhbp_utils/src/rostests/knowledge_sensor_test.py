#!/usr/bin/env python
"""
Tests the knowledge sensor.
Requires a running roscore

@author: rieger, hrabia
"""
import time
import unittest

import rospy
import rostest
import roslaunch
from rhbp_utils.knowledge_sensors import KnowledgeSensor, KnowledgeFactNumberSensor, KnowledgeFactCountSensor
from knowledge_base.knowledge_base_client import KnowledgeBaseClient, KnowledgeBase

PKG = 'rhbp_utils'

"""
Integration test for various KnowledgeSensor classes.
"""


class TestKnowledgeBaseSensor(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestKnowledgeBaseSensor, self).__init__(*args, **kwargs)

        self.__knowledge_base_address = KnowledgeBase.DEFAULT_NAME + "KnowledgeBaseSensorTestSuite"

        # prevent influence of previous tests
        self.__message_prefix = 'TestKnowledgeBaseSensor' + str(time.time())

        self.start_kb_node()

        rospy.init_node('knowledge_sensor_test_node', log_level=rospy.DEBUG)
        self.__client = KnowledgeBaseClient(knowledge_base_name=self.__knowledge_base_address)

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
        rospy.sleep(1)

    def test_basic(self):
        """
        Tests sensor output, if the fact is added at runtime (and did not exist before)
        """
        sensor = KnowledgeSensor(pattern=(self.__message_prefix, 'test_basic', 'pos', '*', '*'),
                                 knowledge_base_name=self.__knowledge_base_address)
        rospy.sleep(0.1)
        sensor.sync()
        self.assertFalse(sensor.value)

        update_stamp = sensor._value_cache.update_time
        self.__client.push((self.__message_prefix, 'test_basic', 'pos', '42', '0'))
        rospy.sleep(0.1)
        while update_stamp == sensor._value_cache.update_time:
            rospy.sleep(0.1)

        sensor.sync()
        self.assertTrue(sensor.value)

    def test_remove(self):
        """
        Tests sensor output , if the fact is removed
        """
        test_tuple = (self.__message_prefix, 'test_remove', 'pos', '42', '0')

        sensor = KnowledgeSensor(pattern=(self.__message_prefix, 'test_remove', 'pos', '*', '*'),
                                 knowledge_base_name=self.__knowledge_base_address)
        rospy.sleep(0.1)
        update_stamp = sensor._value_cache.update_time
        self.__client.push(test_tuple)
        rospy.sleep(0.1)
        while update_stamp == sensor._value_cache.update_time:
            rospy.sleep(0.1)
        sensor.sync()
        self.assertTrue(sensor.value)

        update_stamp = sensor._value_cache.update_time

        removed_facts = self.__client.pop(test_tuple)

        self.assertIsNotNone(removed_facts, "No fact removed")

        while update_stamp == sensor._value_cache.update_time:
            rospy.sleep(0.1)

        sensor.sync()

        self.assertFalse(sensor.value)

    def test_knowledge_fact_int_sensor(self):
        """
        Test KnowledgeFactIntSensor
        """

        initial_value = 1337
        sensor_pattern = (self.__message_prefix, 'test_knowledge_fact_int_sensor', 'number', '*')
        sensor = KnowledgeFactNumberSensor(pattern=sensor_pattern, initial_value=initial_value,
                                           knowledge_base_name=self.__knowledge_base_address)
        rospy.sleep(0.1)

        sensor.sync()
        self.assertEquals(sensor.value, initial_value)

        test_value = 42

        update_stamp = sensor._value_cache.update_time
        # regular operation
        self.__client.push((self.__message_prefix, 'test_knowledge_fact_int_sensor', 'number', str(test_value)))
        rospy.sleep(0.1)
        while update_stamp == sensor._value_cache.update_time:
            rospy.sleep(0.1)

        sensor.sync()
        self.assertEquals(sensor.value, test_value)

        update_stamp = sensor._value_cache.update_time
        # illegal operation with non integer value
        new_tuple = (self.__message_prefix, 'test_knowledge_fact_int_sensor', 'number', "NO_NUMBER")
        self.assertEquals(self.__client.update(pattern=sensor_pattern, new=new_tuple), True)
        rospy.sleep(0.1)
        while update_stamp > sensor.value_update_time:
            rospy.sleep(0.5)

        sensor.sync()
        rospy.loginfo(sensor.value)
        self.assertEquals(sensor.value, initial_value)

    def test_knowledge_fact_count_sensor(self):
        """
        Test KnowledgeFactCountSensor
        """

        initial_value = 0
        sensor_pattern = (self.__message_prefix, 'test_knowledge_fact_count_sensor', 'test', '*')
        sensor = KnowledgeFactCountSensor(pattern=sensor_pattern, knowledge_base_name=self.__knowledge_base_address,
                                          initial_value=initial_value)
        rospy.sleep(0.1)

        sensor.sync()
        self.assertEquals(sensor.value, initial_value)

        update_stamp = sensor._value_cache.update_time
        # regular operation
        self.__client.push((self.__message_prefix, 'test_knowledge_fact_count_sensor', 'test', 'a'))
        rospy.sleep(0.1)
        while update_stamp == sensor._value_cache.update_time:
            rospy.sleep(0.1)

        sensor.sync()
        self.assertEquals(sensor.value, 1)

        update_stamp = sensor._value_cache.update_time
        # regular operation
        self.__client.push((self.__message_prefix, 'test_knowledge_fact_count_sensor', 'test', 'b'))
        rospy.sleep(0.1)
        while update_stamp == sensor._value_cache.update_time:
            rospy.sleep(0.1)

        sensor.sync()
        self.assertEquals(sensor.value, 2)

        update_stamp = sensor._value_cache.update_time
        # regular operation
        self.__client.pop((self.__message_prefix, 'test_knowledge_fact_count_sensor', 'test', 'b'))
        rospy.sleep(0.1)
        while update_stamp == sensor._value_cache.update_time:
            rospy.sleep(0.1)

        sensor.sync()
        self.assertEquals(sensor.value, 1)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'knowledge_sensor_test_node', TestKnowledgeBaseSensor)
