#! /usr/bin/env python2
"""
Tests manager implementation and integration

Created on 27.03.2017

@author: hrabia
"""
import time
import unittest

import rospy
import rostest
from behaviour_components.activators import BooleanActivator, GreedyActivator
from behaviour_components.conditions import Condition
from behaviour_components.goals import GoalBase
from behaviour_components.managers import Manager
from behaviour_components.condition_elements import Effect
from behaviour_components.sensors import SimpleTopicSensor, Sensor
from std_msgs.msg import Bool, Int32

from tests.common import SetTrueBehavior, IncreaserBehavior

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
        goal.add_condition(condition)

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
        goal.add_condition(non_interruptable_condition)
        goal.add_condition(interruptable_condition)

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

    def test_handle_interfering_correlations(self):
        """
        Test manager interference resolving capabilities
        """

        method_prefix = self.__message_prefix + "test_handle_interfering_correlations"
        planner_prefix = method_prefix + "Manager"
        m = Manager(activationThreshold=7.0, prefix=planner_prefix)

        topic_name_1 = method_prefix + '/sensor_1'
        sensor1 = SimpleTopicSensor(topic=topic_name_1, message_type=Int32, initial_value=False)
        condition1 = Condition(sensor1, GreedyActivator())
        condition_function_name = condition1.getFunctionNames()[0]
        behaviour1 = IncreaserBehavior(effect_name=condition_function_name, topic_name=topic_name_1,
                        name=method_prefix + "TopicIncreaser", plannerPrefix=planner_prefix, interruptable=True)

        topic_name_2 = method_prefix + '/sensor_2'
        sensor2 = SimpleTopicSensor(topic=topic_name_2, message_type=Int32, initial_value=False)
        condition2 = Condition(sensor2, GreedyActivator())
        condition_function_name2 = condition2.getFunctionNames()[0]
        behaviour2 = IncreaserBehavior(effect_name=condition_function_name2, topic_name=topic_name_2,
                        name=method_prefix + "TopicIncreaser2", plannerPrefix=planner_prefix, interruptable=True)

        # add a conflict here "-1"
        behaviour1.add_effect(Effect(sensor_name=condition_function_name2, indicator=-1, sensor_type=int))

        goal = GoalBase(method_prefix + 'CentralGoal', plannerPrefix=planner_prefix, permanent=True)
        goal.add_condition(condition1)
        goal.add_condition(condition2)

        # first one of the behaviours can not be executed due to a conflict
        for x in range(0, 4, 1):
            m.step()
            rospy.sleep(0.1)
        # behaviour 2 should be executed because it does not conflict
        self.assertFalse(behaviour1._isExecuting, "Behaviour 1 is executed in spite of a conflict")
        self.assertTrue(behaviour2._isExecuting, "Behaviour 2 is not executed")

        behaviour1.priority = 1  # increase priority

        for x in range(0, 1, 1):
            m.step()
            rospy.sleep(0.1)

        # now behaviour 1 should be executed
        self.assertTrue(behaviour1._isExecuting, "Behaviour 1 is not executed in spite of higher priority")
        self.assertFalse(behaviour2._isExecuting, "Behaviour 2 is executed in spite of lower priority")

        behaviour1.priority = 0  # reset priority

        # Manipulate activation of behaviour 2 with an extra goal, wish is positively influencing
        goal2 = GoalBase(method_prefix + 'ExtraGoal', plannerPrefix=planner_prefix, permanent=True)
        goal2.add_condition(condition2)  # this condition is targeted by behaviour 2

        rospy.sleep(0.1)

        for x in range(0, 5, 1):  # it takes some time with configured decay factor to integrate the changed activation
            m.step()
            rospy.sleep(0.1)

        self.assertFalse(behaviour1._isExecuting, "Behaviour 1 is executed")
        self.assertTrue(behaviour2._isExecuting, "Behaviour 2 is not executed")


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_goals_node', TestManager)
    rospy.spin()
