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
from behaviour_components.activators import Condition, ThresholdActivator, BooleanActivator, GreedyActivator
from behaviour_components.goals import  GoalBase
from behaviour_components.managers import Manager
from behaviour_components.sensors import SimpleTopicSensor, Sensor
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
        """
        Test behaviour property independentFromPlanner
        """

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
        goal_proxy.fetchStatus(3)
        self.assertTrue(goal_proxy.satisfied, 'Goal is not satisfied')

        self.assertTrue(independent_behaviour._isExecuting, "independent_behaviour is not executed")
        self.assertFalse(independent_behaviour2.was_executed, "independent_behaviour2 was executed")
        self.assertFalse(independent_behaviour3.was_executed, "independent_behaviour2 was executed")

    def test_interruptable_behaviour(self):
        """
        Test behaviour interruptable property
        """

        method_prefix = self.__message_prefix + "test_interruptable_behaviour"
        planner_prefix = method_prefix + "Manager"
        m = Manager(activationThreshold=7, prefix=planner_prefix)

        topic_name_1 = method_prefix + '/sensor_1'
        non_interruptable_sensor = SimpleTopicSensor(topic=topic_name_1, message_type=Int32, initial_value=False)
        non_interruptable_condition = Condition(non_interruptable_sensor, GreedyActivator())
        condition_function_name = non_interruptable_condition.getFunctionNames()[0]
        non_interruptable_behaviour = IncreaserBehavior(effect_name=condition_function_name, topic_name=topic_name_1,
                        name=method_prefix + "TopicIncreaser", plannerPrefix=planner_prefix, interruptable=False)

        topic_name_2 = method_prefix + '/sensor_2'
        interruptable_sensor = SimpleTopicSensor(topic=topic_name_2, message_type=Int32, initial_value=False)
        interruptable_condition = Condition(interruptable_sensor, GreedyActivator())
        condition_function_name2 = interruptable_condition.getFunctionNames()[0]
        interruptable_behaviour = IncreaserBehavior(effect_name=condition_function_name2, topic_name=topic_name_2,
                        name=method_prefix + "TopicIncreaser2", plannerPrefix=planner_prefix, interruptable=True)

        enable_sensor = Sensor(name='enable_sensor', initial_value=True)
        enable_cond = Condition(enable_sensor, BooleanActivator())
        non_interruptable_behaviour.addPrecondition(enable_cond)

        goal = GoalBase(method_prefix + 'CentralGoal', plannerPrefix=planner_prefix, permanent=True)
        goal.addCondition(non_interruptable_condition)
        goal.addCondition(interruptable_condition)

        # first normal operation: every behaviour runs as expected
        for x in range(0, 4, 1):
            m.step()
            rospy.sleep(0.1)

        self.assertTrue(non_interruptable_behaviour._isExecuting, "Non-Interruptable Behaviour is not executed")
        self.assertTrue(interruptable_behaviour._isExecuting, "Interruptable Behaviour is not executed")

        #disable non_interruptable_behaviour precondition and check if it is affected
        enable_sensor.update(False)
        for x in range(0, 1, 1):
            m.step()
            rospy.sleep(0.1)

        self.assertTrue(non_interruptable_behaviour._isExecuting, "Non-Interruptable Behaviour is not executed")
        self.assertTrue(interruptable_behaviour._isExecuting, "Interruptable Behaviour is not executed")

        #disable precondition of interruptable behaviour and check if it is disabled as well

        interruptable_behaviour.addPrecondition(enable_cond)

        for x in range(0, 1, 1):
            m.step()
            rospy.sleep(0.1)

        self.assertTrue(non_interruptable_behaviour._isExecuting, "Non-Interruptable Behaviour is not executed")
        self.assertFalse(interruptable_behaviour._isExecuting, "Interruptable Behaviour is executed")


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_goals_node', TestManager)
    rospy.spin()
