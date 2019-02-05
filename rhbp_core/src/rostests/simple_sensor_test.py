#! /usr/bin/env python2
"""
@author: hrabia
"""

import unittest
import rostest

import rospy
from std_msgs.msg import Int32, ColorRGBA
from rhbp_core.msg import PlannerStatus, Status

from behaviour_components.sensors import TopicSensor, AggregationSensor

PKG = 'rhbp_core'


class SimpleSensorTestSuite(unittest.TestCase):
    """Testing rhbp sensors"""

    def __init__(self, *args, **kwargs):
        super(SimpleSensorTestSuite, self).__init__(*args, **kwargs)
        rospy.init_node('SensorTestSuiteNode', log_level=rospy.WARN)

    def test_attribute_access(self):
        """
        Test the access to individual attributes
        """

        #test default config accessing data attribute
        primitive_topic = '/primitive_topic'
        pub_primitive = rospy.Publisher(primitive_topic, Int32, queue_size=1, latch=True)

        #test access to particular attribute of a message
        complex_topic = '/complex_topic'
        pub_complex = rospy.Publisher(complex_topic, ColorRGBA, queue_size=1, latch=True)
        rospy.sleep(0.1)

        test_value = 1337

        pub_primitive.publish(test_value)
        pub_complex.publish(ColorRGBA(g=test_value))

        primitive_sensor = TopicSensor(topic=primitive_topic)

        complex_sensor = TopicSensor(topic=complex_topic, message_attr='g')

        rospy.sleep(0.1)

        primitive_sensor.sync()
        complex_sensor.sync()

        self.assertEquals(test_value, primitive_sensor.value, "Primitive test value does not match")
        self.assertEquals(test_value, complex_sensor.value, "Complex test value does not match")

    def test_nested_and_key_attribute_access(self):
        """
        Test the access to individual attributes that are nested within a hierarchy of types or are elements of a list,
        tuple etc.
        """

        #test access to particular nested attribute of a message
        complex_topic = '/complex_nested_type_topic'
        pub_complex = rospy.Publisher(complex_topic, PlannerStatus, queue_size=1, latch=True)
        rospy.sleep(0.1)

        test_value = 1337
        test_value2 = "second"
        test_initial_value = "initial"

        msg = PlannerStatus()

        msg.plan.append("first")
        msg.plan.append(test_value2)

        msg.goals.append(Status(activation=test_value))

        pub_complex.publish(msg)

        complex_sensor = TopicSensor(topic=complex_topic, message_attr='plan[1]', initial_value=test_initial_value)

        very_complex_sensor = TopicSensor(topic=complex_topic, message_attr='goals[0].activation')

        rospy.sleep(0.1)

        very_complex_sensor.sync()
        complex_sensor.sync()

        self.assertEquals(test_value2, complex_sensor.value, "Complex test value does not match")

        self.assertEquals(test_value, very_complex_sensor.value, "Complex iterated nested test value does not match")

        # empty list test
        msg = PlannerStatus()
        pub_complex.publish(msg)
        rospy.sleep(0.1)

        complex_sensor.sync()

        self.assertEquals(test_initial_value, complex_sensor.value, "Complex sensor value does not match initial value")

    def test_simple_sensor_aggregation(self):
        """
        Test an aggregation of simple topics with the AggregationSensor
        """

        primitive_topic_1 = '/primitive_topic1'
        pub_primitive_1 = rospy.Publisher(primitive_topic_1, Int32, queue_size=1, latch=True)
        primitive_topic_2 = '/primitive_topic2'
        pub_primitive_2 = rospy.Publisher(primitive_topic_2, Int32, queue_size=1, latch=True)

        rospy.sleep(0.1)

        test_value_1 = 3
        test_value_2 = 7

        pub_primitive_1.publish(test_value_1)
        pub_primitive_2.publish(test_value_2)

        primitive_sensor_1 = TopicSensor(topic=primitive_topic_1)
        primitive_sensor_2 = TopicSensor(topic=primitive_topic_2)

        # wait until the topic callbacks have been processed
        while primitive_sensor_1.latestValue is None or primitive_sensor_2.latestValue is None:
            rospy.sleep(0.1)

        def aggregation_function(sensor_values):
            return sum(sensor_values)

        aggregation_sensor = AggregationSensor(name="aggr", sensors=[primitive_sensor_1, primitive_sensor_2],
                                               func=aggregation_function)

        aggregation_sensor.sync()

        self.assertEquals(test_value_1 + test_value_2, aggregation_sensor.value, "Aggregation value does not match")

        # Testing alternative way of using the AggregationSensor with Inheritance

        aggregation_sensor = AggregationSensor(name="aggr", sensors=[primitive_sensor_1, primitive_sensor_2],
                                               func=None)

        self.assertRaises(NotImplementedError, aggregation_sensor.sync)

        class MyAggregationSensor(AggregationSensor):
            def __init__(self, name, sensors, optional=False, initial_value=None):
                super(MyAggregationSensor, self).__init__(name=name, sensors=sensors, optional=optional,
                                                          initial_value=initial_value)

            def _aggregate(self, sensor_values):
                return sum(sensor_values)

        aggregation_sensor = MyAggregationSensor(name="aggr", sensors=[primitive_sensor_1, primitive_sensor_2])

        aggregation_sensor.sync()

        self.assertEquals(test_value_1 + test_value_2, aggregation_sensor.value, "Aggregation value does not match")


if __name__ == '__main__':
    rostest.rosrun(PKG, 'SimpleSensorTestNode', SimpleSensorTestSuite)
