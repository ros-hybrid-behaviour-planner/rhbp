#!/usr/bin/env python
"""
Tests the param sensor and behaviour
Requires a running roscore

@author: hrabia
"""
import unittest

import rospy
import rostest
import time
import roslaunch
from rhbp_utils.param_sensors import ParamSensor
from rhbp_utils.param_behaviour import ParamSetterBehaviour

PKG = 'rhbp_utils'

"""
Integration test for ROS param utils.
"""


class TestParamUtils(unittest.TestCase):

    manager_name = "TestParamUtils_manager"

    def __init__(self, *args, **kwargs):
        super(TestParamUtils, self).__init__(*args, **kwargs)

        self._manager_prefix = "TestParamUtils_manager" + str(time.time()).replace('.', '')

        rospy.init_node('param_utils_test_node', log_level=rospy.DEBUG)

    def start_manager_node(self, prefix=""):
        """
        start the manager node in order to successfully launch behaviours
        """
        package = 'rhbp_core'
        executable = 'planner_node.py'
        args = "prefix:=" + prefix
        node = roslaunch.core.Node(package=package, node_type=executable, name=self.manager_name,
                                   output='screen', args=args)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        self._manager_process = launch.launch(node)

    def setUp(self):
        self.param_name_int = "/global/test/parameter_int"
        self.param_name_bool = "/global/test/parameter_bool"
        self.param_name_str = "/global/test/parameter_str"

        # reset test parameters
        if rospy.has_param(self.param_name_int):
            rospy.delete_param(self.param_name_int)
        if rospy.has_param(self.param_name_bool):
            rospy.delete_param(self.param_name_bool)
        if rospy.has_param(self.param_name_str):
            rospy.delete_param(self.param_name_str)

        self.start_manager_node(prefix=self._manager_prefix)

    def tearDown(self):
        self._manager_process.stop()

    def test_param_sensor(self):
        """
        Testing parameter sensors
        """

        # Test 1 parameter not available

        sensor_int = ParamSensor(name="test_int", param=self.param_name_int, initial_value=None)

        sensor_int.sync()

        self.assertEquals(sensor_int.value, None)

        # Test 2 int param

        test_value = 2
        rospy.set_param(self.param_name_int, test_value)

        sensor_int.sync()

        self.assertEquals(sensor_int.value, test_value)

        # Test 3 bool param

        test_value_bool = True
        rospy.set_param(self.param_name_bool, test_value_bool)

        sensor_bool = ParamSensor(name="test_bool", param=self.param_name_bool, initial_value=False)

        sensor_bool.sync()

        self.assertEquals(sensor_bool.value, test_value_bool)

        # Test 4 string param

        test_value_str = "TEST"
        rospy.set_param(self.param_name_str, test_value_str)

        sensor_str = ParamSensor(name="test_str", param=self.param_name_str, initial_value=False)

        sensor_str.sync()

        self.assertEquals(sensor_str.value, test_value_str)

    def test_param_setter(self):
        """
        Testing generic parameter setting behaviour
        """

        test_value = rospy.get_param(self.param_name_bool, False)

        self.assertEquals(False, test_value)

        behaviour = ParamSetterBehaviour(name="Test", param=self.param_name_bool, new_value=True,
                                         planner_prefix=self._manager_prefix)
        # force manual start
        behaviour.start()

        test_value = rospy.get_param(self.param_name_bool)

        self.assertEquals(True, test_value)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'param_utils_test_node', TestParamUtils)
