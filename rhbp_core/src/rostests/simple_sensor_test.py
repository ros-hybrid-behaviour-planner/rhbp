#! /usr/bin/env python2
'''
@author: hrabia
'''

import unittest
import rostest

import rospy
from std_msgs.msg import Int32, ColorRGBA

from behaviour_components.sensors import SimpleTopicSensor

PKG = 'rhbp_core'

class SimpleSensorTestSuite(unittest.TestCase):
    """Testing rhbp sensors"""

    def __init__(self, *args, **kwargs):
        super(SimpleSensorTestSuite, self).__init__(*args, **kwargs)
        rospy.init_node('SensorTestSuiteNode', log_level=rospy.WARN)

    def test_attribut_access(self):
        """
        Test the access to individual attributes
        """

        #test default config accessing data attribute
        primitive_topic = '/primitive_topic'
        pub_primitive = rospy.Publisher(primitive_topic, Int32, queue_size=10, latch=True)

        #test access to particular attribute of a message
        complex_topic='/complex_topic'
        pub_complex = rospy.Publisher(complex_topic, ColorRGBA, queue_size=10, latch=True)
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


if __name__ == '__main__':
    rostest.rosrun(PKG, 'SimpleSensorTestNode', SimpleSensorTestSuite)