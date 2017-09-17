#! /usr/bin/env python2
'''
Tests the goal implementations

Created on 15.12.2016

@author: rieger
'''
import time
import unittest

import rospy
import rostest

from std_msgs.msg import Bool

from behaviour_components.activators import Condition, BooleanActivator
from behaviour_components.goals import OfflineGoal, GoalBase
from behaviour_components.managers import Manager
from behaviour_components.sensors import SimpleTopicSensor

from tests.common import SetTrueBehavior

PKG = 'rhbp_core'

"""
System test for goals. Assumes, that a rosmaster is running
"""


class TestGoals(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestGoals, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TestGoals' + str(time.time()).replace('.', '')
        rospy.init_node('goal_test_node', log_level=rospy.DEBUG)
        # Disable planner, since the change from python to C
        #  disturbs the connection between the test process and the node process
        rospy.set_param("~planBias", 0.0)

    def test_remote_goal(self):

        method_prefix = self.__message_prefix + "TestRemoteGoal"
        planner_prefix = method_prefix + "Manager"
        m = Manager(activationThreshold=7, prefix=planner_prefix)

        topic_name = method_prefix + '/Topic'

        sensor = SimpleTopicSensor(topic=topic_name, message_type=Bool, initial_value=False)
        condition = Condition(sensor, BooleanActivator())

        pddl_function_name = condition.getFunctionNames()[0]
        SetTrueBehavior(effect_name=pddl_function_name, topic_name=topic_name,
                        name=method_prefix + "SetTrue", plannerPrefix=planner_prefix)
        goal = GoalBase(method_prefix + 'CentralGoal', plannerPrefix=planner_prefix)
        goal.addCondition(condition)

        for x in range(0, 3, 1):
            m.step()
            rospy.sleep(0.1)

        goal_proxy = m.goals[0]
        goal_proxy.fetchStatus(3)
        self.assertTrue(goal_proxy.satisfied, 'Goal is not satisfied')

    def test_offline_goal(self):

        method_prefix = self.__message_prefix + "TestOfflineGoal"
        planner_prefix = method_prefix + "/Manager"
        m = Manager(activationThreshold=7, prefix=planner_prefix)

        topic_name = method_prefix + '/Topic'

        sensor = SimpleTopicSensor(topic=topic_name, message_type=Bool, initial_value=False)
        condition = Condition(sensor, BooleanActivator())

        pddl_function_name = condition.getFunctionNames()[0]
        SetTrueBehavior(effect_name=pddl_function_name, topic_name=topic_name,
                        name=method_prefix + "SetTrue", plannerPrefix=planner_prefix)
        goal = OfflineGoal('CentralGoal', planner_prefix=planner_prefix)
        goal.add_condition(condition)
        m.add_goal(goal)
        for x in range(0, 3, 1):
            m.step()
            rospy.sleep(0.1)

        goal.fetchStatus(3)

        self.assertTrue(goal.satisfied, 'Goal is not satisfied')

        goal.unregister()


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_goals_node', TestGoals)
    rospy.spin()
