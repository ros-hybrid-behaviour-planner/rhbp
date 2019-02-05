#! /usr/bin/env python2
"""
Tests condition elements

Created on 23.08.2017

@author: hrabia
"""

from __future__ import division  # force floating point division when using plain /

import unittest

import rospy
import rostest

from behaviour_components.activators import LinearActivator
from behaviour_components.conditions import Condition
from behaviour_components.managers import Manager
from behaviour_components.sensors import Sensor
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect

PKG = 'rhbp_core'

"""
Testing condition elements, requires running roscore
"""


class TestConditionElements(unittest.TestCase):
    """
    Testing condition elements
    TODO this test is not yet complete
    """

    def __init__(self, *args, **kwargs):
        super(TestConditionElements, self).__init__(*args, **kwargs)
        rospy.init_node('condition_elements_test_node', log_level=rospy.DEBUG)

    def test_inverted_activation(self):

        planner_prefix = "condition_elements_test"
        m = Manager(activationThreshold=7, prefix=planner_prefix, createLogFiles=True)

        satisfaction_threshold = 0.8

        sensor1 = Sensor()

        activator_increasing = LinearActivator(zeroActivationValue=0, fullActivationValue=1)
        activator_decreasing = LinearActivator(zeroActivationValue=1, fullActivationValue=0)
        condition_increasing = Condition(sensor1, activator_increasing)
        condition_decreasing = Condition(sensor1, activator_decreasing)

        sensor1.update(newValue=0.8)

        condition_increasing.sync()
        condition_increasing.updateComputation()
        condition_decreasing.sync()
        condition_decreasing.updateComputation()

        wish_increasing = condition_increasing.getWishes()[0]
        wish_decreasing = condition_decreasing.getWishes()[0]

        self.assertAlmostEqual( 0.2, wish_increasing.indicator, delta=0.001)
        self.assertAlmostEqual(-0.8, wish_decreasing.indicator, delta=0.001)

        increasing_precondition_precon_pddl = condition_increasing.getPreconditionPDDL(satisfaction_threshold=satisfaction_threshold)
        decreasing_precondition_precon_pddl = condition_decreasing.getPreconditionPDDL(satisfaction_threshold=satisfaction_threshold)

        increasing_precondition_state_pddl = condition_increasing.getStatePDDL()[0]
        decreasing_precondition_state_pddl = condition_decreasing.getStatePDDL()[0]

        sensor2 = Sensor()
        sensor2.update(newValue=0.4)

        behaviour1 = BehaviourBase("behaviour_1", planner_prefix=planner_prefix)

        behaviour1.add_effect(Effect(sensor_name=sensor1.name,indicator=-0.1, sensor_type=float))

        behaviour1.add_precondition(condition_increasing)

        behaviour2 = BehaviourBase("behaviour_2", planner_prefix=planner_prefix)

        behaviour2.add_effect(Effect(sensor_name=sensor1.name, indicator=0.1, sensor_type=float))

        behaviour2.add_precondition(condition_decreasing)

        m.step()
        m.step()

        b1_state_pddl = behaviour1.getStatePDDL()
        b2_state_pddl = behaviour2.getStatePDDL()

        b1_action_pddl = behaviour1.getActionPDDL()
        b2_action_pddl = behaviour2.getActionPDDL()

        for x in range(0, 3, 1):
            m.step()
            rospy.sleep(0.1)
        #
        # goal_proxy = m.goals[0]
        # goal_proxy.sync()
        # self.assertTrue(goal_proxy.satisfied, 'Goal is not satisfied')


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_condition_elements_node', TestConditionElements)
