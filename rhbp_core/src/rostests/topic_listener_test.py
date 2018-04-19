#!/usr/bin/env python
"""
Tests the topic listener.
Requires a running topic listener node

@author: rieger
"""
import time
import unittest

import rospy
import rostest
import roslaunch
from rhbp_core.srv import TopicUpdateSubscribe
from std_msgs.msg import Int32

from behaviour_components.sensors import DynamicSensor
from utils.topic_listener import TopicListener

PKG = 'rhbp_core'

"""
System test for the topic listener (including the DynamicSensor)
"""


class MaxValueSensor(DynamicSensor):
    def __init__(self, pattern_prefix):
        super(MaxValueSensor, self).__init__(pattern=pattern_prefix, initial_value=Int32(0))

    def _aggregate_values(self, values):
        max_value = 0
        for v in values:
            max_value = max(max_value, v.data)

        return max_value


class TopicListenerTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TopicListenerTest, self).__init__(*args, **kwargs)


    def setUp(self):
        # prevent influence of previous tests
        self.__message_prefix = 'TopicListenerTestSuite_' + str(int(time.time()))

        #start topic listener node
        node = roslaunch.core.Node(package='rhbp_core', node_type='topic_listener.py', name='TopicListenerNode')
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        self._kb_process = launch.launch(node)

        #init test node
        rospy.init_node('TopicListenerTestNode', log_level=rospy.DEBUG)
        self.__subscribe_service = rospy.ServiceProxy(
            TopicListener.DEFAULT_NAME + TopicListener.SUBSCRIBE_SERVICE_NAME_POSTFIX, TopicUpdateSubscribe)

    def tearDown(self):
        self._kb_process.stop()

    @staticmethod
    def create_topic(topic_name):
        pub = rospy.Publisher(topic_name, Int32, queue_size=10)
        rospy.sleep(1.0)
        return pub

    def test_basic(self):
        """
        Tests sensor output, if topic is added
        """
        prefix = '/' + self.__message_prefix + 'testBasic'
        sensor = MaxValueSensor(pattern_prefix=prefix)
        sensor.sync()
        self.assertEqual(0, sensor.value, 'Initial value is not correct')

        topic1 = TopicListenerTest.create_topic(prefix + 'IntTest1')
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(0, sensor.value, 'Value has changed unexpected')

        topic1.publish(1)
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(1, sensor.value, 'Value has not changed')

        topic2 = TopicListenerTest.create_topic(prefix + 'AnyTopic2')
        rospy.sleep(0.1)
        topic2.publish(2)
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(2, sensor.value, 'Seccond value was not passed')

    def test_existing(self):
        prefix = '/' + self.__message_prefix + 'testExisting'
        topic1 = TopicListenerTest.create_topic(prefix + 'IntTest1')

        sensor = MaxValueSensor(pattern_prefix=prefix)
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(0, sensor.value, 'Initial value is not correct')

        topic1.publish(1)
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(1, sensor.value, 'Value has not changed')

    def test_several_sensors(self):
        """
        Tests subscribing of several sensors
        """
        prefix = '/' + self.__message_prefix + 'testSeveralSensors'
        sensor1 = MaxValueSensor(pattern_prefix=prefix)
        sensor2 = MaxValueSensor(pattern_prefix=prefix)
        sensor1.sync()
        sensor2.sync()
        self.assertEqual(0, sensor1.value, 'Initial value of sensor1 is not correct')
        self.assertEqual(0, sensor2.value, 'Initial value of sensor2 is not correct')

        TopicListenerTest.create_topic(prefix + 'IntTest1')
        rospy.sleep(0.1)
        sensor1.sync()
        sensor2.sync()
        self.assertEqual(0, sensor1.value, 'Value has changed unexpected')
        self.assertEqual(0, sensor2.value, 'Value has changed unexpected')


if __name__ == '__main__':
    rostest.rosrun(PKG, 'topic_listener_test_node', TopicListenerTest)
