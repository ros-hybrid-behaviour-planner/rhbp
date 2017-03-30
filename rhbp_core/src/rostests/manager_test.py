#! /usr/bin/env python2
'''
Tests manager implementation and integration

Created on 27.03.2017

@author: hrabia
'''
import time
import unittest

import rospy
import rostest
from behaviour_components.activators import Condition, ThresholdActivator, BooleanActivator
from behaviour_components.goals import  GoalBase
from behaviour_components.managers import Manager
from behaviour_components.sensors import SimpleTopicSensor
from std_msgs.msg import Bool, Int32

from tests.common import SetTrueBehavior,IncreaserBehavior

PKG = 'rhbp_core'

"""
System test for manager and its integration with other components
"""


class TestManager(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestManager, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TestManager_' + str(time.time()).replace('.', '')
        rospy.init_node('manager_test_node', log_level=rospy.DEBUG)

    def test_independent_behaviour(self):

        method_prefix = self.__message_prefix + "test_independent_behaviour"
        planner_prefix = method_prefix + "Manager"
        m = Manager(activationThreshold=7, prefix=planner_prefix)

        topic_name_1 = method_prefix + '/sensor_1'

        sensor = SimpleTopicSensor(topic=topic_name_1, message_type=Bool, initial_value=False)
        condition = Condition(sensor, BooleanActivator())

        condition_function_name = condition.getFunctionNames()[0]
        #independentFromPlanner and effects
        independent_behaviour = SetTrueBehavior(effect_name=condition_function_name, topic_name=topic_name_1,
                        name=method_prefix + "SetTrue", plannerPrefix=planner_prefix, independentFromPlanner=True)

        #independentFromPlanner and no effects
        independent_behaviour2 = SetTrueBehavior(effect_name=None, topic_name=topic_name_1,
                                                name=method_prefix + "SetTrue2", plannerPrefix=planner_prefix,
                                                independentFromPlanner=True)
        # not independentFromPlanner and no effects
        independent_behaviour3 = SetTrueBehavior(effect_name=None, topic_name=topic_name_1,
                                                 name=method_prefix + "SetTrue3", plannerPrefix=planner_prefix,
                                                 independentFromPlanner=False)

        goal = GoalBase(method_prefix + 'CentralGoal', plannerPrefix=planner_prefix)
        goal.addCondition(condition)

        for x in range(0, 3, 1):
            m.step()
            rospy.sleep(0.1)

        goal_proxy = m.goals[0]
        goal_proxy.sync()
        self.assertTrue(goal_proxy.satisfied, 'Goal is not satisfied')

        self.assertTrue(independent_behaviour._isExecuting, "independent_behaviour is not executed")
        self.assertFalse(independent_behaviour2.was_executed, "independent_behaviour2 was executed")
        self.assertFalse(independent_behaviour3.was_executed, "independent_behaviour2 was executed")

    def test_interruptable_behaviour(self):

        #TODO

        method_prefix = self.__message_prefix + "test_interruptable_behaviour"
        planner_prefix = method_prefix + "Manager"
        m = Manager(activationThreshold=7, prefix=planner_prefix)

        topic_name_1 = method_prefix + '/sensor_1'

        sensor = SimpleTopicSensor(topic=topic_name_1, message_type=Int32, initial_value=0)
        condition = Condition(sensor, ThresholdActivator(thresholdValue=3))

        condition_function_name = condition.getFunctionNames()[0]
        independent_behaviour = IncreaserBehavior(effect_name=condition_function_name, topic_name=topic_name_1,
                        name=method_prefix + "TopicIncreaser", plannerPrefix=planner_prefix, interruptable=False)

        #TODO add precondition to force stop

        goal = GoalBase(method_prefix + 'CentralGoal', plannerPrefix=planner_prefix)
        goal.addCondition(condition)

        for x in range(0, 5, 1):
            m.step()
            rospy.sleep(0.1)

        goal_proxy = m.goals[0]
        goal_proxy.sync()
        self.assertTrue(goal_proxy.satisfied, 'Goal is not satisfied')

        self.assertTrue(independent_behaviour._isExecuting, "Independent Behaviour is not executed")


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_goals_node', TestManager)
    rospy.spin()
