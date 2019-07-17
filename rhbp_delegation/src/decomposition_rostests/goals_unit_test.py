#! /usr/bin/env python2
"""
Unit tests for the decomposition goal

Needs a running ROSCORE!

@author: Mengers
"""

import unittest
import rospy
import rostest
from behaviour_components.managers import Manager
from behaviour_components.sensors import TopicSensor
from behaviour_components.conditions import Condition
from behaviour_components.activators import BooleanActivator
from std_msgs.msg import Bool
from decomposition_components.goal_wrapper import DecompositionGoal
from delegation_module_tests.test_utils import MockedDelegationCommunicator
from decomposition_components.delegation_goal import DelegationGoal


class DecompositionGoalTest(unittest.TestCase):
    """
    Unit tests for the DecompositionGoal
    """

    def setUp(self):
        rospy.init_node("TestNode")
        sensor = TopicSensor(name="test_sensor", topic="sensor_topic", message_type=Bool, initial_value=False)
        self.conditions = [Condition(sensor, BooleanActivator())]
        self.manager_name = "Test_manager"
        self.manager = Manager(prefix=self.manager_name)
        self.goal_name = "test_goal"
        self.satisfaction_threshold = 1.0

    def tearDown(self):
        self.manager.unregister()

    def test_check_alive(self):
        """
        Tests the check_if_alive function
        """

        uut = DecompositionGoal(name=self.goal_name, planner_prefix=self.manager_name, conditions=self.conditions, satisfaction_threshold=self.satisfaction_threshold)
        self.assertTrue(uut.check_if_alive())
        self.manager.unregister()
        self.assertFalse(uut.check_if_alive())
        uut.unregister()


class DelegationGoalTest(unittest.TestCase):
    """
    Unit tests for the DelegationGoal
    """

    def setUp(self):
        rospy.init_node("TestNode")
        sensor = TopicSensor(name="test_sensor", topic="sensor_topic", message_type=Bool, initial_value=False)
        self.conditions = [Condition(sensor, BooleanActivator())]
        self.manager_name = "Test_manager"
        self.manager = Manager(prefix=self.manager_name)
        self.goal_name = "test_goal"
        self.satisfaction_threshold = 1.0
        self.mockedDM = MockedDelegationCommunicator(manager_name=self.manager_name, name=self.manager_name)
        self.goal_repr = " ".join([x.getPreconditionPDDL(1.0).statement for x in self.conditions])

    def tearDown(self):
        self.mockedDM.__del__()
        self.manager.unregister()

    def test_start_auction(self):
        """
        Tests start auction
        """

        test_goal = DelegationGoal(name=self.goal_name, conditions=self.conditions)
        self.assertFalse(test_goal.contractor_found)
        self.assertFalse(test_goal.auction_running)
        test_goal.start_auction()
        rospy.sleep(2)
        self.assertTrue(test_goal.auction_running)
        self.assertFalse(test_goal.contractor_found)
        self.assertTrue(self.mockedDM.got_cfp)
        self.assertEqual(self.mockedDM.CFP_last.name, "/TestNode")
        self.assertEqual(self.mockedDM.CFP_last.goal_representation, self.goal_repr)
        test_goal.unregister()

    def test_start_contractor(self):
        """
        Tests start_with_contractor
        """

        test_goal = DelegationGoal(name=self.goal_name, conditions=self.conditions)
        self.assertFalse(test_goal.contractor_found)
        test_goal.start_with_contractor(planner_prefix=self.manager_name)
        self.assertTrue(test_goal.contractor_found)
        self.assertEqual(self.manager.goals[0].name, self.goal_name)
        test_goal.unregister()

    def test_finish_auction(self):
        """
        Tests finish auction
        """

        test_goal = DelegationGoal(name=self.goal_name, conditions=self.conditions)
        test_goal.start_auction()
        self.mockedDM.send_propose(3.0, "/TestNode", test_goal.delegation_id)
        self.mockedDM.set_precom_response(acceptance=True, still_bidding=True, cost=3.0)
        self.assertTrue(test_goal.finish_auction())
        self.assertTrue(test_goal.contractor_found)
        self.assertEqual(self.manager.goals[0].name, self.goal_name)
        test_goal.unregister()


if __name__ == '__main__':
    #rostest.rosrun('rhbp_decomposition', 'test_delegation_goal_node', DelegationGoalTest)
    rostest.rosrun('rhbp_decomposition', 'test_decomposition_goal_node', DecompositionGoalTest)
    rospy.spin()
