#! /usr/bin/env python2
"""
@author: hrabia
"""

import unittest
import rostest

import rospy
from std_msgs.msg import Int32, ColorRGBA
from rhbp_core.msg import PlannerStatus, Status

from behaviour_components.sensors import SimpleTopicSensor

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

        primitive_sensor = SimpleTopicSensor(topic=primitive_topic)

        complex_sensor = SimpleTopicSensor(topic=complex_topic, message_attr='g')

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

        msg = PlannerStatus()

        msg.plan.append("first")
        msg.plan.append(test_value2)

        msg.goals.append(Status(activation=test_value))

        pub_complex.publish(msg)

        complex_sensor = SimpleTopicSensor(topic=complex_topic, message_attr='plan[1]')

        very_complex_sensor = SimpleTopicSensor(topic=complex_topic, message_attr='goals[0].activation')

        rospy.sleep(0.1)

        very_complex_sensor.sync()
        complex_sensor.sync()

        self.assertEquals(test_value2, complex_sensor.value, "Complex test value does not match")

        self.assertEquals(test_value, very_complex_sensor.value, "Complex iterated nested test value does not match")


if __name__ == '__main__':
    rostest.rosrun(PKG, 'SimpleSensorTestNode', SimpleSensorTestSuite)