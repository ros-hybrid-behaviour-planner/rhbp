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
from behaviour_components.sensors import DynamicSensor
from behaviour_components.topic_listener import TopicListener
from std_msgs.msg import Int32

from rhbp_core.srv import TopicUpdateSubscribe

PKG = 'rhbp_core'


class MaxValueSensor(DynamicSensor):
    def __init__(self, pattern_prefix):
        super(MaxValueSensor, self).__init__(pattern=pattern_prefix, default_value=Int32(0))

    def _aggregate_values(self, values):
        max_value = 0
        for v in values:
            max_value = max(max_value, v.data)

        return max_value


class TopicListenerTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TopicListenerTest, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TopicListenerTestSuite' + str(int(time.time()))
        rospy.init_node('TopicListenerTestNode', log_level=rospy.DEBUG)
        self.__subscribe_service = rospy.ServiceProxy(
            TopicListener.DEFAULT_NODE_NAME + TopicListener.SUBSCRIBE_SERVICE_NAME_POSTFIX, TopicUpdateSubscribe)

    @staticmethod
    def create_topic(topic_name):
        """
        Adds the given fact to knowledge base
        :param to_add: array of strings
        """
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


if __name__ == '__main__':
    rostest.rosrun(PKG, 'topic_listener_test_node', TopicListenerTest)
