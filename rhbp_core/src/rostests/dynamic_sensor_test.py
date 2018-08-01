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
from rhbp_core.srv import TopicUpdateSubscribe, TopicUpdateSubscribeResponse
from std_msgs.msg import String, Int32

from behaviour_components.sensors import DynamicSensor
from utils.topic_listener import TopicListener

PKG = 'rhbp_core'

"""
Tests the integration of the DynamicSensor into ROS
"""


class MaxValueSensor(DynamicSensor):
    def __init__(self, pattern_prefix, service_prefix, expiration_time_values_of_removed_topics=1000, initial_value=0,
                 topic_type=None):
        super(MaxValueSensor, self).__init__(pattern=pattern_prefix,
                                             topic_listener_name=service_prefix,
                                             expiration_time_values_of_removed_topics=expiration_time_values_of_removed_topics,
                                             initial_value=initial_value, topic_type=topic_type)
        self.last_received_topic = None

    def _aggregate_values(self, values):
        if (not values):
            return super(MaxValueSensor, self)._aggregate_values([])

        max_value = 0
        for v in values:
            max_value = max(max_value, v.data)

        return max_value

    def _value_updated(self, topic, value, time_stamp):
        self.last_received_topic = topic
        super(MaxValueSensor, self)._value_updated(topic, value, time_stamp)

    @property
    def last_value(self):
        return self._last_value

    @property
    def min_value(self):
        return self._min_value

    @property
    def max_value(self):
        return self._max_value

    def _convert_ros_message_to_output_format(self, message):
        return message.data


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
            return TopicUpdateSubscribeResponse(topicNameTopicAdded=topic_names[0].name,
                                                topicNameTopicRemoved=topic_names[1].name,
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
        rospy.sleep(0.1)
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
    def create_topic(topic_name, topic_type=Int32):
        pub = rospy.Publisher(topic_name, topic_type, queue_size=10)
        rospy.sleep(0.1)
        return pub

    @staticmethod
    def create_topic_and_publish(topic_listener, name, first_value, topic_type=Int32):
        topic = DynamicSensorTest.create_topic(name, topic_type=topic_type)
        topic_listener.add_topic(name)
        rospy.sleep(0.1)
        topic.publish(first_value)
        rospy.sleep(0.1)
        return topic

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
        self.assertIsNone(sensor.last_received_topic)

        topic1 = DynamicSensorTest.create_topic(prefix + 'IntTest1')
        topic_listener.add_topic(prefix + 'IntTest1')
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(0, sensor.value, 'Value has changed unexpected')
        self.assertIsNone(sensor.last_received_topic)

        topic1.publish(1)
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(1, sensor.value, 'Value has not changed')
        self.assertEqual(topic1.name, sensor.last_received_topic)

        topic2 = DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'anyTopic2', 2)
        sensor.sync()
        self.assertEqual(2, sensor.value, 'Seccond value was not passed')
        self.assertEqual(topic2.name, sensor.last_received_topic)

        topic_listener.remove_topic(prefix + 'anyTopic2')
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(1, sensor.value, 'value of seccond topic was removed, but sensor value has not changed')
        self.assertEqual(topic2.name, sensor.last_received_topic)

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

    def test_default_aggregation(self):
        """
        Tests default implementation of aggregation (just use last received value)
        """
        prefix = '/' + self.__message_prefix + 'testDefaultAggreagtion'
        service_prefix = prefix + 'Service'
        topic_listener = TopicListenerMock(service_prefix=service_prefix)
        sensor = DynamicSensor(pattern=prefix, optional=False, initial_value=0,
                               topic_listener_name=service_prefix)
        DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'Topic1', 1)
        topic2 = DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'Topic2', 2)

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
        sensor = DynamicSensor(pattern=prefix, optional=False, initial_value=0,
                               topic_listener_name=service_prefix, subscribe_only_first=True)
        DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'Topic1', 1)
        DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'Topic2', 2)

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

    def test_default_value(self):
        default_value = 42
        prefix = '/' + self.__message_prefix + 'testDefaultValue'
        service_prefix = prefix + 'Service'
        topic_listener = TopicListenerMock(service_prefix=service_prefix)
        sensor = MaxValueSensor(pattern_prefix=prefix, initial_value=default_value,
                                service_prefix=service_prefix, expiration_time_values_of_removed_topics=0)
        sensor.sync()
        self.assertEqual(default_value, sensor.value, 'the default value is not used')

        topic = DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'Topic1', 1)

        sensor.sync()
        self.assertEqual(1, sensor.value, 'Value was not received, it is instead: ' + str(sensor.value))

        topic_listener.remove_topic(topic.name)
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(1, sensor.value,
                         'The latest value is not used after removing last topic, it is instead: ' + str(sensor.value))

    def test_aggregation_help_values(self):
        """
        Tests updating of last value
        """
        prefix = '/' + self.__message_prefix + 'testAggregationHelpValues'
        service_prefix = prefix + 'Service'
        topic_listener = TopicListenerMock(service_prefix=service_prefix)
        sensor = MaxValueSensor(pattern_prefix=prefix, service_prefix=service_prefix,
                                expiration_time_values_of_removed_topics=0)
        sensor.sync()
        self.assertEqual(None, sensor.last_value)
        self.assertIsNone(sensor.last_received_topic)

        topic1 = DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'Topic1', 1)
        sensor.sync()
        self.assertEqual(1, sensor.value, 'Value was not received, it is instead: ' + str(sensor.value))
        self.assertEqual(Int32(1), sensor.last_value, 'Latest value was not updated')
        self.assertEqual(topic1.name, sensor.last_received_topic)

        topic2 = DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + 'Topic2', 2)
        sensor.sync()
        self.assertEqual(2, sensor.value, 'Second value was not received, it is instead: ' + str(sensor.value))
        self.assertEqual(Int32(2), sensor.last_value, 'Second value is not latest value')
        self.assertEqual(topic2.name, sensor.last_received_topic)

        topic_listener.remove_topic(topic2.name)
        rospy.sleep(0.1)
        sensor.sync()
        self.assertEqual(1, sensor.value, 'Second topic was removed, but value is still present')
        self.assertEqual(Int32(2), sensor.last_value, 'Latest value shouldnt change after removing topic')
        self.assertEqual(topic2.name, sensor.last_received_topic)

        topic1.publish(0)
        rospy.sleep(0.1)

        self.assertEqual(0, sensor.min_value.data)
        self.assertEqual(2, sensor.max_value.data)

    def test_topic_type_filtering(self):
        prefix = '/' + self.__message_prefix + 'testTopicType'
        service_prefix = prefix + 'Service'
        topic_listener = TopicListenerMock(service_prefix=service_prefix)
        sensor = DynamicSensor(pattern=prefix,
                               topic_listener_name=service_prefix,
                               expiration_time_values_of_removed_topics=-1,
                               initial_value=Int32(-1), topic_type=Int32)
        sensor.sync()
        self.assertEqual(Int32(-1), sensor.value)

        DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + '/First/ToSubscribe', 1, topic_type=Int32)
        sensor.sync()
        self.assertEqual(Int32(1), sensor.value, 'Value was not received, it is instead: ' + str(sensor.value))

        DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + '/WrongType', 'MyFamousMessage',
                                                   topic_type=String)
        sensor.sync()
        self.assertEqual(Int32(1), sensor.value, 'Value has changed unexpectedly: ' + str(sensor.value))

        DynamicSensorTest.create_topic_and_publish(topic_listener, prefix + '/Second/ToSubscribe', 42, topic_type=Int32)
        sensor.sync()
        self.assertEqual(Int32(42), sensor.value, 'Value was not received, it is instead: ' + str(sensor.value))


if __name__ == '__main__':
    rostest.rosrun(PKG, 'dynamic_sensor_test_node', DynamicSensorTest)
