#!/usr/bin/env python
"""
Tests the dynamic sensor.

@author: rieger
"""
import re
import time
import unittest

import rospy
import rostest
from behaviour_components.sensors import DynamicSensor
from behaviour_components.topic_listener import TopicListener
from std_msgs.msg import String, Int32

from rhbp_core.srv import TopicUpdateSubscribe, TopicUpdateSubscribeResponse

PKG = 'rhbp_core'

"""
Tests the integration of the DynamicSensor into ROS
"""


class MaxValueSensor(DynamicSensor):
    def __init__(self, pattern_prefix, service_prefix, expiration_time_values_of_removed_topics=1000):
        super(MaxValueSensor, self).__init__(pattern=pattern_prefix, default_value=Int32(0),
                                             topic_listener_name=service_prefix,
                                             expiration_time_values_of_removed_topics=expiration_time_values_of_removed_topics)

    def _aggregate_values(self, values):
        max_value = 0
        for v in values:
            max_value = max(max_value, v.data)

        return max_value


class TopicListenerMock(object):
    def __init__(self, service_prefix):
        self.__subscribe_service = rospy.Service(service_prefix + TopicListener.SUBSCRIBE_SERVICE_NAME_POSTFIX,
                                                 TopicUpdateSubscribe,
                                                 self.__subscribe_callback)
        self.__prefix = service_prefix
        self.__topic_counter = 0
        self.__existing_topics = []
        self.__subscribed_expressions = []
        self.__update_topics = {}

    def __del__(self):
        """
            closes all services
        """
        self.__subscribe_service.shutdown()

    def __subscribe_callback(self, request):
        pattern = request.regex
        regex = re.compile(pattern)
        if (regex in self.__update_topics):
            topic_names = self.__update_topics[regex]
            return TopicUpdateSubscribeResponse(topicNameTopicAdded=topic_names[0],
                                                topicNameTopicRemoved=topic_names[1],
                                                existingTopics=self.__find_matching_topcis(regex))

        self.__subscribed_expressions.append(regex)
        base_topic_name = TopicListener.generate_topic_name_for_pattern(self.__prefix + '/Topics/', pattern,
                                                                        True,
                                                                        self.__topic_counter)
        self.__topic_counter += 1
        added_topic_name = base_topic_name + '/TopicAdded'
        added_topic = rospy.Publisher(added_topic_name, String, queue_size=10)
        removed_topic_name = base_topic_name + '/TopicRemoved'
        removed_topic = rospy.Publisher(removed_topic_name, String, queue_size=10)
        rospy.sleep(1)
        self.__update_topics[regex] = (added_topic, removed_topic)
        return TopicUpdateSubscribeResponse(topicNameTopicAdded=added_topic_name,
                                            topicNameTopicRemoved=removed_topic_name,
                                            existingTopics=self.__find_matching_topcis(regex))

    def __find_matching_topcis(self, regex):
        matching_topics = []
        for topic_name in self.__existing_topics:
            if (regex.match(topic_name)):
                matching_topics.append(topic_name)
        return tuple(matching_topics)

    def add_topic(self, name):
        self.__existing_topics.append(name)
        for regex in self.__subscribed_expressions:
            if regex.match(name):
                self.__update_topics[regex][0].publish(name)

    def remove_topic(self, name):
        if (name in self.__existing_topics):
            self.__existing_topics.remove(name)
        for regex in self.__subscribed_expressions:
            if regex.match(name):
                self.__update_topics[regex][1].publish(name)


class DynamicSensorTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(DynamicSensorTest, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TopicListenerTestSuite' + str(int(time.time()))
        rospy.init_node('DynamicSensorTestNode', log_level=rospy.DEBUG)
        self.__subscribe_service = rospy.ServiceProxy(
            TopicListener.DEFAULT_NAME + TopicListener.SUBSCRIBE_SERVICE_NAME_POSTFIX, TopicUpdateSubscribe)

    @staticmethod
    def create_topic(topic_name):
        """
        Adds the given fact to knowledge base
        :param to_add: array of strings
        """
        pub = rospy.Publisher(topic_name, Int32, queue_size=10)
        rospy.sleep(0.1)
        return pub

    def test_basic(self):
        """
        Tests sensor output, if topic is added
        """
        prefix = '/' + self.__message_prefix + 'testBasic'
        service_prefix = prefix + 'Service'
        topic_listener = TopicListenerMock(service_prefix=service_prefix)
        sensor = MaxValueSensor(pattern_prefix=prefix, service_prefix=service_prefix,
                                expiration_time_values_of_removed_topics=0)
        sensor.sync()
        self.assertEqual(0, sensor.value, 'Initial value is not correct')

        topic1 = DynamicSensorTest.create_topic(prefix + 'IntTest1')
        topic_listener.add_topic(prefix + 'IntTest1')
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(0, sensor.value, 'Value has changed unexpected')

        topic1.publish(1)
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(1, sensor.value, 'Value has not changed')

        topic2 = DynamicSensorTest.create_topic(prefix + 'anyTopic2')
        topic_listener.add_topic(prefix + 'anyTopic2')
        rospy.sleep(0.1)
        topic2.publish(2)
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(2, sensor.value, 'Seccond value was not passed')

        topic_listener.remove_topic(prefix + 'anyTopic2')
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(1, sensor.value, 'value of seccond topic was removed, but sensor value has not changed')

    def test_existing(self):
        """
        Tests the detection of values, which published in a topic, which does already exist at subscribing
        """
        prefix = '/' + self.__message_prefix + 'testExisting'
        service_prefix = prefix + 'Service'
        topic_listener = TopicListenerMock(service_prefix=service_prefix)
        topic1 = DynamicSensorTest.create_topic(prefix + 'IntTest1')
        topic_listener.add_topic(prefix + 'IntTest1')

        rospy.sleep(0.1)
        sensor = MaxValueSensor(pattern_prefix=prefix, service_prefix=service_prefix)
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(0, sensor.value, 'Initial value is not correct')

        topic1.publish(1)
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(1, sensor.value, 'Value has not changed')

    @staticmethod
    def create_topic_and_publish(topic_listener, name, first_value):
        topic = DynamicSensorTest.create_topic(name)
        topic_listener.add_topic(name)
        rospy.sleep(0.1)
        topic.publish(first_value)
        rospy.sleep(0.1)
        return topic

    def test_default_aggregation(self):
        """
        Tests default implementation of aggregation (just use last received value)
        """
        prefix = '/' + self.__message_prefix + 'testDefaultAggreagtion'
        service_prefix = prefix + 'Service'
        topic_listener = TopicListenerMock(service_prefix=service_prefix)
        sensor = DynamicSensor(pattern=prefix, optional=False, default_value=Int32(0),
                               topic_listener_name=service_prefix)
        DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'Topic1', 1)
        topic2 = DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'Topic2', 2)

        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(2, sensor.value.data)

        topic_listener.remove_topic(topic2.name)

        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(2, sensor.value.data)

    def test_subscribing_limit(self):
        """
        Checks, that only the first matching pattern is used, if the subcribe_only_first flag was set
        """
        prefix = '/' + self.__message_prefix + 'testSubscribingLimit'
        service_prefix = prefix + 'Service'
        topic_listener = TopicListenerMock(service_prefix=service_prefix)
        sensor = DynamicSensor(pattern=prefix, optional=False, default_value=Int32(0),
                               topic_listener_name=service_prefix, subscribe_only_first=True)
        DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'Topic1', 1)
        DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'Topic2', 2)

        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(1, sensor.value.data)

    def test_value_removing(self):
        """
        Checks, that outdated values are not used for value calculation
        """
        prefix = '/' + self.__message_prefix + 'testTreeshold'
        service_prefix = prefix + 'Service'
        topic_listener = TopicListenerMock(service_prefix=service_prefix)
        sensor = MaxValueSensor(pattern_prefix=prefix, service_prefix=service_prefix,
                                expiration_time_values_of_removed_topics=1)

        DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'IntTest1', 10)
        topic2 = DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'IntTest2', 20)

        sensor.sync()
        self.assertEqual(20, sensor.value, 'Value has not changed')

        topic_listener.remove_topic(topic2.name)

        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(20, sensor.value, 'Value of removed topic is removed, but should not')

        rospy.sleep(1)
        sensor.sync()
        self.assertEqual(10, sensor.value, 'Value of removed topic should expired, but is not')


if __name__ == '__main__':
    rostest.rosrun(PKG, 'dynamic_sensor_test_node', DynamicSensorTest)
