#! /usr/bin/env python2
"""
@author: hrabia
"""

import unittest
import rostest

import rospy
from std_msgs.msg import Int32, ColorRGBA

from behaviour_components.sensors import SimpleTopicSensor, ParamSensor

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

    def test_param_sensor(self):
        param_name_int = "/global/test/parameter_int"
        param_name_bool = "/global/test/parameter_bool"
        param_name_str = "/global/test/parameter_str"

        # reset test
        if rospy.has_param(param_name_int):
            rospy.delete_param(param_name_int)
        if rospy.has_param(param_name_bool):
            rospy.delete_param(param_name_bool)
        if rospy.has_param(param_name_str):
            rospy.delete_param(param_name_str)

        # Test 1 parameter not available

        sensor_int = ParamSensor(name="test_int", param=param_name_int, initial_value=None)

        sensor_int.sync()

        self.assertEquals(sensor_int.value, None)

        # Test 2 int param

        test_value = 2
        rospy.set_param(param_name_int, test_value)

        sensor_int.sync()

        self.assertEquals(sensor_int.value, test_value)

        # Test 3 bool param

        test_value_bool = True
        rospy.set_param(param_name_bool, test_value_bool)

        sensor_bool = ParamSensor(name="test_bool", param=param_name_bool, initial_value=False)

        sensor_bool.sync()

        self.assertEquals(sensor_bool.value, test_value_bool)

        # Test 4 string param

        test_value_str = "TEST"
        rospy.set_param(param_name_str, test_value_str)

        sensor_str = ParamSensor(name="test_str", param=param_name_str, initial_value=False)

        sensor_str.sync()

        self.assertEquals(sensor_str.value, test_value_str)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'SimpleSensorTestNode', SimpleSensorTestSuite)