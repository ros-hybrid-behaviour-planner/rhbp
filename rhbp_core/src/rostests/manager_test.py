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
from behaviour_components.activators import BooleanActivator, GreedyActivator, ThresholdActivator
from behaviour_components.conditions import Condition, Conjunction
from behaviour_components.goals import GoalBase
from behaviour_components.managers import Manager
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect
from behaviour_components.sensors import TopicSensor, Sensor
from rhbp_core.srv import PlanWithGoal
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

        sensor = TopicSensor(topic=topic_name_1, message_type=Bool, initial_value=False)
        condition = Condition(sensor, BooleanActivator())

        #independentFromPlanner and effects
        independent_behaviour = SetTrueBehavior(effect_name=sensor.name, topic_name=topic_name_1,
                        name=method_prefix + "SetTrue", planner_prefix=planner_prefix, independentFromPlanner=True)

        #independentFromPlanner and no effects
        independent_behaviour2 = SetTrueBehavior(effect_name=None, topic_name=topic_name_1,
                                                name=method_prefix + "SetTrue2", planner_prefix=planner_prefix,
                                                independentFromPlanner=True)
        # not independentFromPlanner and no effects
        independent_behaviour3 = SetTrueBehavior(effect_name=None, topic_name=topic_name_1,
                                                 name=method_prefix + "SetTrue3", planner_prefix=planner_prefix,
                                                 independentFromPlanner=False)

        goal = GoalBase(method_prefix + 'CentralGoal', planner_prefix=planner_prefix)
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
        non_interruptable_sensor = TopicSensor(topic=topic_name_1, message_type=Int32, initial_value=False)
        non_interruptable_condition = Condition(non_interruptable_sensor, GreedyActivator())
        non_interruptable_behaviour = IncreaserBehavior(effect_name=non_interruptable_sensor.name, topic_name=topic_name_1,
                        name=method_prefix + "TopicIncreaser", planner_prefix=planner_prefix, interruptable=False)

        topic_name_2 = method_prefix + '/sensor_2'
        interruptable_sensor = TopicSensor(topic=topic_name_2, message_type=Int32, initial_value=False)
        interruptable_condition = Condition(interruptable_sensor, GreedyActivator())
        interruptable_behaviour = IncreaserBehavior(effect_name=interruptable_sensor.name, topic_name=topic_name_2,
                        name=method_prefix + "TopicIncreaser2", planner_prefix=planner_prefix, interruptable=True)

        enable_sensor = Sensor(name='enable_sensor', initial_value=True)
        enable_cond = Condition(enable_sensor, BooleanActivator())
        non_interruptable_behaviour.add_precondition(enable_cond)

        goal = GoalBase(method_prefix + 'CentralGoal', planner_prefix=planner_prefix, permanent=True)
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

        interruptable_behaviour.add_precondition(enable_cond)

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
        sensor1 = TopicSensor(topic=topic_name_1, message_type=Int32, initial_value=False)
        condition1 = Condition(sensor1, GreedyActivator())
        behaviour1 = IncreaserBehavior(effect_name=sensor1.name, topic_name=topic_name_1,
                        name=method_prefix + "TopicIncreaser", planner_prefix=planner_prefix, interruptable=True)

        topic_name_2 = method_prefix + '/sensor_2'
        sensor2 = TopicSensor(topic=topic_name_2, message_type=Int32, initial_value=False)
        condition2 = Condition(sensor2, GreedyActivator())
        behaviour2 = IncreaserBehavior(effect_name=sensor2.name, topic_name=topic_name_2,
                        name=method_prefix + "TopicIncreaser2", planner_prefix=planner_prefix, interruptable=True)

        # add a conflict here "-1"
        behaviour1.add_effect(Effect(sensor_name=sensor2.name, indicator=-1, sensor_type=int))

        goal = GoalBase(method_prefix + 'CentralGoal', planner_prefix=planner_prefix, permanent=True)
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
        goal2 = GoalBase(method_prefix + 'ExtraGoal', planner_prefix=planner_prefix, permanent=True)
        goal2.add_condition(condition2)  # this condition is targeted by behaviour 2

        rospy.sleep(0.1)

        for x in range(0, 5, 1):  # it takes some time with configured decay factor to integrate the changed activation
            m.step()
            rospy.sleep(0.1)

        self.assertFalse(behaviour1._isExecuting, "Behaviour 1 is executed")
        self.assertTrue(behaviour2._isExecuting, "Behaviour 2 is not executed")

    def test_plan_with_registered_goals(self):

        method_prefix = self.__message_prefix + "test_handle_interfering_correlations"
        planner_prefix = method_prefix + "Manager"
        manager = Manager(activationThreshold=7.0, prefix=planner_prefix)

        sensor_1 = Sensor(name="Sensor1", initial_value=False)
        sensor_2 = Sensor(name="Sensor2", initial_value=False)
        sensor_3 = Sensor(name="Sensor3", initial_value=False)

        behaviour_1 = BehaviourBase(name="Behaviour1", planner_prefix=planner_prefix)
        behaviour_1.add_effect(Effect(sensor_name=sensor_1.name, indicator=1))
        behaviour_2 = BehaviourBase(name="Behaviour2", planner_prefix=planner_prefix)
        behaviour_2.add_effect(Effect(sensor_name=sensor_2.name, indicator=1))
        behaviour_2.add_precondition(Condition(sensor_1, BooleanActivator()))
        behaviour_3 = BehaviourBase(name="Behaviour3", planner_prefix=planner_prefix)
        behaviour_3.add_effect(Effect(sensor_name=sensor_3.name, indicator=1))
        behaviour_3.add_precondition(Condition(sensor_2, BooleanActivator()))

        goal1 = GoalBase(name="Test_Goal1", conditions=[Condition(sensor_3, BooleanActivator())],
                         planner_prefix=planner_prefix)

        goal2 = GoalBase(name="Test_Goal2", conditions=[Condition(sensor_2, BooleanActivator())],
                         planner_prefix=planner_prefix)

        # test plannning without prior decision step
        plan_seq = manager.plan_with_registered_goals([manager.goals[0]])

        expected_plan_seq = ["Behaviour1", "Behaviour2", "Behaviour3"]

        self.assertEquals(len(plan_seq), len(expected_plan_seq))
        self.assertListEqual(plan_seq, expected_plan_seq)

        # plan again using a service

        service_name = planner_prefix + '/' + 'PlanWithGoal'
        # do not wait forever here, manager might be already closed
        rospy.wait_for_service(service_name, timeout=1)
        plan_service = rospy.ServiceProxy(service_name, PlanWithGoal)

        # use all registered goals
        res = plan_service()

        self.assertEquals(len(res.plan_sequence), len(expected_plan_seq))
        self.assertListEqual(res.plan_sequence, expected_plan_seq)

        # use other named goal
        expected_plan_seq = ["Behaviour1", "Behaviour2"]
        res = plan_service(goal_names=[goal2.name])

        self.assertEquals(len(res.plan_sequence), len(expected_plan_seq))
        self.assertListEqual(res.plan_sequence, expected_plan_seq)

        # test infeasible plan

        sensor_4 = Sensor(name="Sensor4", initial_value=False)

        goal3 = GoalBase(name="Test_Goal3", conditions=[Condition(sensor_4, BooleanActivator())],
                         planner_prefix=planner_prefix)

        expected_plan_seq = []

        # force an update to make the new goal available
        res = plan_service(goal_names=[goal3.name], force_state_update=True)

        self.assertEquals(len(res.plan_sequence), len(expected_plan_seq))
        self.assertListEqual(res.plan_sequence, expected_plan_seq)

    def test_guarantee_decision(self):
        """
        Testing guarantee_decision parameter of manager.step()
        """

        method_prefix = self.__message_prefix + "test_guarantee_decision"
        planner_prefix = method_prefix + "Manager"
        manager = Manager(activationThreshold=7.0, prefix=planner_prefix)

        sensor_3 = Sensor(name="Sensor3", initial_value=False)

        behaviour_1 = BehaviourBase(name="Behaviour1", planner_prefix=planner_prefix)

        goal1 = GoalBase(name="Test_Goal1", conditions=[Condition(sensor_3, BooleanActivator())],
                         planner_prefix=planner_prefix)

        manager.step()
        self.assertFalse(behaviour_1._isExecuting,
                         "Behaviour 1 is executed even though it should not have enough activation")

        manager.step(guarantee_decision=True)
        self.assertTrue(behaviour_1._isExecuting,
                        "Behaviour 1 is not executed even though a decision was forced")

    def test_plan_monitoring(self):
        """
        Testing plan monitoring and replanning management
        """

        method_prefix = self.__message_prefix + "test_plan_monitoring"
        planner_prefix = method_prefix + "Manager"
        manager = Manager(activationThreshold=7.0, prefix=planner_prefix, max_parallel_behaviours=1)

        sensor_1 = Sensor(name="Sensor1", initial_value=False)

        behaviour_1 = BehaviourBase(name="Behaviour1", planner_prefix=planner_prefix)
        behaviour_1.add_effect(Effect(sensor_1.name, 1))

        sensor_2 = Sensor(name="Sensor2", initial_value=False)

        behaviour_2 = BehaviourBase(name="Behaviour2", planner_prefix=planner_prefix)
        behaviour_2.add_effect(Effect(sensor_2.name, 1))

        behaviour_3 = BehaviourBase(name="Behaviour3", planner_prefix=planner_prefix)
        # adding precondition to get a reference to the sensor in the manager
        behaviour_3.add_precondition(Condition(sensor_2, BooleanActivator()))
        behaviour_3.add_effect(Effect(sensor_2.name, 1))

        goal1 = GoalBase(name="Test_Goal1", conditions=[Conjunction(
                         Condition(sensor_1, BooleanActivator(desiredValue=True)),
                         Condition(sensor_2, BooleanActivator(desiredValue=False),
                        ))], planner_prefix=planner_prefix, permanent=True)

        manager.step(guarantee_decision=True)
        self.assertTrue(behaviour_1._isExecuting,
                        "Behaviour 1 is not executed even though a decision was forced and it would fulfill the plan")

        sensor_1.update(True)  # faking effect of behaviour1, all changes are induced by the behaviour, no replanning

        manager.step(guarantee_decision=True)
        self.assertTrue(behaviour_1._isExecuting,
                        "Behaviour 1 is not executed even though a decision was forced and it would fulfill the plan")
        self.assertTrue(manager._are_all_sensor_changes_from_executed_behaviours())
        manager._planExecutionIndex -= 1  # we have to manipulate here because it was incremented
        self.assertTrue(manager._are_effects_of_planned_behaviour_realised())
        manager._planExecutionIndex += 1

        sensor_2.update(True)  # faking additional external effect

        manager.step(guarantee_decision=True)
        self.assertTrue(behaviour_1._isExecuting,
                        "Behaviour 1 is not executed even though a decision was forced and it would fulfill the plan")
        self.assertFalse(manager._are_all_sensor_changes_from_executed_behaviours())
        self.assertTrue(manager._executed_behaviours_influenced_as_expected())
        self.assertFalse(manager._are_effects_of_planned_behaviour_realised())

        sensor_1.update(False)

        manager.step(guarantee_decision=True)
        self.assertTrue(behaviour_1._isExecuting,
                        "Behaviour 1 is not executed even though a decision was forced and it would fulfill the plan")
        # behaviour1 should set sensor1 to True if it is running but the contrary happened
        self.assertFalse(manager._executed_behaviours_influenced_as_expected())

    def test_plan_monitoring_float_sensors(self):
        """
        Testing plan monitoring and replanning management
        """

        method_prefix = self.__message_prefix + "test_plan_monitoring_float_sensors"
        planner_prefix = method_prefix + "Manager"
        manager = Manager(activationThreshold=7.0, prefix=planner_prefix, max_parallel_behaviours=1)

        sensor_1 = Sensor(name="Sensor1", initial_value=0)

        behaviour_1 = BehaviourBase(name="Behaviour1", planner_prefix=planner_prefix)
        behaviour_1.add_effect(Effect(sensor_1.name, 1, sensor_type=float))

        sensor_2 = Sensor(name="Sensor2", initial_value=0)

        behaviour_2 = BehaviourBase(name="Behaviour2", planner_prefix=planner_prefix)
        behaviour_2.add_effect(Effect(sensor_2.name, 1, sensor_type=float))

        behaviour_3 = BehaviourBase(name="Behaviour3", planner_prefix=planner_prefix)
        # adding precondition to get a reference to the sensor in the manager
        behaviour_3.add_precondition(Condition(sensor_2, BooleanActivator()))
        behaviour_3.add_effect(Effect(sensor_2.name, 1, sensor_type=float))

        goal1 = GoalBase(name="Test_Goal1", conditions=[Conjunction(
                         Condition(sensor_1, ThresholdActivator(1)),
                         Condition(sensor_2, ThresholdActivator(0),
                        ))], planner_prefix=planner_prefix, permanent=True)

        manager.step(guarantee_decision=True)
        self.assertTrue(behaviour_1._isExecuting,
                        "Behaviour 1 is not executed even though a decision was forced and it would fulfill the plan")

        sensor_1.update(0.5)  # faking partial effect of behaviour1, all changes are induced by behaviour, no replanning

        manager.step(guarantee_decision=True)
        self.assertTrue(behaviour_1._isExecuting,
                        "Behaviour 1 is not executed even though a decision was forced and it would fulfill the plan")
        self.assertTrue(manager._are_all_sensor_changes_from_executed_behaviours())
        self.assertFalse(manager._are_effects_of_planned_behaviour_realised())  # we did not yet reach the full effect
        self.assertTrue(manager._executed_behaviours_influenced_as_expected())

        sensor_1.update(1.0)  # faking partial effect of behaviour1, all changes are induced by behaviour, no replanning

        manager.step(guarantee_decision=True)
        self.assertTrue(behaviour_1._isExecuting,
                        "Behaviour 1 is not executed even though a decision was forced and it would fulfill the plan")
        self.assertTrue(manager._are_all_sensor_changes_from_executed_behaviours())
        self.assertTrue(manager._executed_behaviours_influenced_as_expected())
        manager._planExecutionIndex -= 1  # we have to manipulate here because it was incremented
        self.assertTrue(manager._are_effects_of_planned_behaviour_realised())
        manager._planExecutionIndex += 1


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_goals_node', TestManager)
    rospy.spin()
