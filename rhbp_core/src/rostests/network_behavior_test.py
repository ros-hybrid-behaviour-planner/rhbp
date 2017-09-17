#! /usr/bin/env python2
'''
Tests the goal implementations

Created on 25.03.2017

@author: rieger
'''
import time
import unittest

import rospy
import rostest

from behaviour_components.activators import Condition, ThresholdActivator
from behaviour_components.goals import OfflineGoal
from behaviour_components.managers import Manager
from behaviour_components.network_behavior import NetworkBehavior
from behaviour_components.condition_elements import Effect
from behaviour_components.sensors import SimpleTopicSensor
from std_msgs.msg import Int32

from tests.common import IncreaserBehavior

PKG = 'rhbp_core'

"""
System test for network behavior. Assumes, that a rosmaster is running
"""

class TestNetworkBehavior(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestNetworkBehavior, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TestNetworkBehavior_' + str(time.time()).replace('.', '')
        rospy.init_node('NetworkBehaviorTestNode', log_level=rospy.INFO)
        # Disable planner, since the change from python to C
        #  disturbs the connection between the test process and the node process
        rospy.set_param("~planBias", 0.0)

    def test_multiple_embedded_network_behaviors(self):
        """
        Tests the case, that one network behavior is embedded into another network behavior.
        The goal requires to receive an int (3) in a topic.
        """

        method_prefix = self.__message_prefix + "/test_multiple_embedded_network_behaviors"

        topic_name = method_prefix + '/Topic'
        sensor = SimpleTopicSensor(topic=topic_name, message_type=Int32, initial_value=0)
        condition = Condition(sensor, ThresholdActivator(thresholdValue=3))

        planner_prefix = method_prefix + "/Manager"
        m = Manager(activationThreshold=7, prefix=planner_prefix)
        goal = OfflineGoal('CentralGoal', planner_prefix=planner_prefix)
        goal.add_condition(condition)
        m.add_goal(goal)

        effect = Effect(sensor_name=sensor.name, indicator=1, sensor_type=int, activator_name=condition.activator.name)

        first_level_network = NetworkBehavior(name=method_prefix + '/FirstLevel', plannerPrefix=planner_prefix, createLogFiles=True)
        first_level_network.add_effects_and_goals([(sensor, effect)])

        second_level_network = NetworkBehavior(name=method_prefix + '/SecondLevel',
                                                plannerPrefix=first_level_network.get_manager_prefix(), createLogFiles=True)
        # Doesnt matter, whether the effects are added via the constructor or the add method.
        # Both methods are used here, to demonstrate both ways.
        second_level_network.add_effects_and_goals([(sensor, effect)])

        pddl_function_name = condition.getFunctionNames()[0]
        increaser_behavior = IncreaserBehavior(effect_name=pddl_function_name, topic_name=topic_name,
                                                    name=method_prefix + "TopicIncreaser",
                                                    plannerPrefix=second_level_network.get_manager_prefix())

        # activate the first_level_network, second_level_network and increaser_Behavior
        for x in range(0, 3, 1):
            self.assertFalse(first_level_network._isExecuting)
            m.step()
            rospy.sleep(0.1)
        self.assertTrue(first_level_network._isExecuting)

        for x in range(0, 3, 1):
            self.assertFalse(second_level_network._isExecuting)
            first_level_network.do_step()
            rospy.sleep(0.1)
        self.assertTrue(second_level_network._isExecuting)

        for x in range(0, 3, 1):
            self.assertFalse(increaser_behavior._isExecuting)
            second_level_network.do_step()
            rospy.sleep(0.1)
        self.assertTrue(increaser_behavior._isExecuting)

        # Satisfy goal
        for step in range(0, 3, 1):
            sensor.sync()
            self.assertEqual(step, sensor.value)
            second_level_network.do_step()
            rospy.sleep(0.1)

        goal.fetchStatus(3)
        self.assertTrue(goal.satisfied, 'Goal is not satisfied')


if __name__ == '__main__':
    rostest.rosrun(PKG, 'NetworkBehaviorTestNode', TestNetworkBehavior)
