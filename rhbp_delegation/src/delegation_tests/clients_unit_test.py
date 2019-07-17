"""
Unit tests for the RHBPDelegationClient variations

Needs a running ROSCORE!

@author: Mengers
"""

from decomposition_components.delegation_clients import RHBPDelegationClient, RHBPDelegableClient
from decomposition_components.manager_client import RHBPManagerDelegationClient
from decomposition_components.cost_computing import PDDLCostEvaluator
from delegation_module_tests.test_utils import MockedDelegationManager
from test_utils import MockedManager, FunctionPointerTester, MockedGoal
import unittest
import rospy


class TestClientSimple(RHBPDelegationClient):
    """
    Version with mocked DelegationManager
    """

    def _create_delegation_manager(self):
        RHBPDelegationClient.used_delegation_manager = MockedDelegationManager()


class TestClientDelegable(RHBPDelegableClient):
    """
    Version with mocked DelegationManager
    """

    def _create_delegation_manager(self):
        RHBPDelegationClient.used_delegation_manager = MockedDelegationManager()


class TestClientManager(RHBPManagerDelegationClient):
    """
    Version with mocked DelegationManager
    """

    def _create_delegation_manager(self):
        RHBPDelegationClient.used_delegation_manager = MockedDelegationManager()

    def _create_own_delegation_manager(self):
        self._delegation_manager = MockedDelegationManager()


class RHBPClientsTest(unittest.TestCase):
    """
    Unit tests for the RHBPDelegationClient variations
    """

    def setUp(self):
        rospy.init_node("TestNode")
        self.mockedDM = None
        self.mockedManager = MockedManager()
        self.checking_prefix = None

    def tearDown(self):
        # make sure no additional stuff from other tests is still here
        RHBPDelegationClient.used_delegation_manager = None
        self.mockedDM = None

    # simple DelegationClient

    def test_delegate(self):
        """
        Tests delegate of the RHBPDelegationClient
        """

        uut = TestClientSimple(checking_prefix=self.checking_prefix)
        self.mockedDM = uut.used_delegation_manager

        goal_name = "test_goal"
        conditions = ["test_conditions"]
        threshold = 0.9

        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold)

        self.assertEqual(delegation_id, 1)
        self.assertEqual(self.mockedDM.goal_wrapper._conditions, conditions)
        self.assertEqual(self.mockedDM.goal_wrapper._satisfaction_threshold, threshold)

    def test_delegation_successful(self):
        """
        Tests the delegation_successful function
        """

        uut = TestClientSimple(checking_prefix=self.checking_prefix)
        self.mockedDM = uut.used_delegation_manager

        # no delegation, no exception
        uut.delegation_successful(delegation_id=1)

        # delegation, no success_function
        goal_name = "test_goal"
        conditions = ["test_conditions"]
        threshold = 0.9

        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold)
        uut.delegation_successful(delegation_id=delegation_id)

        # delegation with success_function
        goal_name = "test_goal"
        conditions = ["test_conditions"]
        threshold = 0.9
        fpt = FunctionPointerTester()

        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold,
                                     success_function=fpt.function)

        uut.delegation_successful(delegation_id=delegation_id)
        self.assertTrue(fpt.function_called)

    def test_terminate_simple(self):
        """
        Tests terminate delegation of simple client
        """

        uut = TestClientSimple(checking_prefix=self.checking_prefix)
        self.mockedDM = uut.used_delegation_manager

        # no delegation, no exception
        uut.terminate_delegation(delegation_id=1)

        # delegation, no success_function
        goal_name = "test_goal"
        conditions = ["test_conditions"]
        threshold = 0.9

        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold)
        uut.terminate_delegation(delegation_id=delegation_id)

        # delegation with success_function
        goal_name = "test_goal"
        conditions = ["test_conditions"]
        threshold = 0.9
        fpt = FunctionPointerTester()

        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold,
                                     success_function=fpt.function)
        uut.terminate_delegation(delegation_id=delegation_id)

        uut.delegation_successful(delegation_id=delegation_id)
        self.assertFalse(fpt.function_called)

    # DelegableClient

    def test_delegable_delegate(self):
        """
        Tests delegable delegate
        """

        uut = TestClientDelegable(checking_prefix=self.checking_prefix)
        self.mockedDM = uut.used_delegation_manager

        goal_name = "test_goal"
        conditions = ["test_conditions"]
        threshold = 0.9

        # base case
        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold)

        self.assertEqual(delegation_id, 1)
        self.assertEqual(self.mockedDM.goal_wrapper._conditions, conditions)
        self.assertEqual(self.mockedDM.goal_wrapper._satisfaction_threshold, threshold)

        # with own cost
        uut = TestClientDelegable(checking_prefix=self.checking_prefix)
        self.mockedDM = uut.used_delegation_manager
        fpt = FunctionPointerTester()
        own_cost = 2.3

        self.assertRaises(RuntimeError, uut.delegate, goal_name=goal_name, conditions=conditions,
                          satisfaction_threshold=threshold, own_cost=own_cost)

        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold,
                                     own_cost=own_cost, start_work_function=fpt.function)

        self.assertEqual(delegation_id, 1)
        self.assertEqual(self.mockedDM.goal_wrapper._conditions, conditions)
        self.assertEqual(self.mockedDM.goal_wrapper._satisfaction_threshold, threshold)
        self.assertEqual(uut._work_function_dictionary[delegation_id], fpt.function)

    def test_work_function_dict(self):
        """
        Tests work functions of DelegableClient
        :return:
        """
        uut = TestClientDelegable(checking_prefix=self.checking_prefix)
        self.mockedDM = uut.used_delegation_manager
        fpt = FunctionPointerTester()
        own_cost = 2.3
        goal_name = "test_goal"
        conditions = ["test_conditions"]
        threshold = 0.9

        self.assertRaises(RuntimeError, uut.delegate, goal_name=goal_name, conditions=conditions,
                          satisfaction_threshold=threshold, own_cost=own_cost)

        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold,
                                     own_cost=own_cost, start_work_function=fpt.function)

        uut.start_work_for_delegation(delegation_id=delegation_id)
        self.assertTrue(fpt.function_called)

        self.assertRaises(KeyError, uut.start_work_for_delegation, delegation_id + 1)

        uut.terminate_delegation(delegation_id=delegation_id)
        self.assertRaises(KeyError, uut.start_work_for_delegation, delegation_id)

    def test_terminate_delegable(self):
        """
        Tests terminate delegation of delegable client
        """

        uut = TestClientDelegable(checking_prefix=self.checking_prefix)
        self.mockedDM = uut.used_delegation_manager

        # no delegation, no exception
        uut.terminate_delegation(delegation_id=1)

        # delegation, no success_function or work function
        goal_name = "test_goal"
        conditions = ["test_conditions"]
        threshold = 0.9

        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold)
        uut.terminate_delegation(delegation_id=delegation_id)

        # delegation with success_function
        goal_name = "test_goal"
        conditions = ["test_conditions"]
        threshold = 0.9
        fpt = FunctionPointerTester()
        work_func = FunctionPointerTester()

        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold,
                                     success_function=fpt.function, own_cost=5, start_work_function=work_func.function)
        uut.terminate_delegation(delegation_id=delegation_id)

        uut.delegation_successful(delegation_id=delegation_id)
        self.assertFalse(fpt.function_called)
        self.assertRaises(KeyError, uut.start_work_for_delegation, delegation_id)

    # Manager Client

    def test_manager_register_without_cost_eval(self):
        """
        Tests manager register, no cost eval
        """

        uut = TestClientManager(manager=self.mockedManager)
        self.mockedDM = uut.used_delegation_manager

        self.assertIsNone(self.mockedDM.cfe)
        self.assertEqual(self.mockedDM.clients[0], uut._client_id)
        self.assertTrue(uut._active_manager)
        self.assertEqual(uut._delegation_manager, self.mockedDM)

    def test_new_cost_evaluator(self):
        """
        Tests cost eval creating
        """

        uut = TestClientManager(manager=self.mockedManager)
        self.mockedDM = uut.used_delegation_manager

        cost_eval = uut.get_new_cost_evaluator()

        self.assertEqual(cost_eval._manager, self.mockedManager)
        self.assertIsInstance(cost_eval, PDDLCostEvaluator)

    def test_manager_register_with_cost_eval(self):
        """
        Tests manager register, cost eval adding
        """

        uut = TestClientManager(manager=self.mockedManager)
        self.mockedDM = uut.used_delegation_manager

        uut.make_cost_computable()

        self.assertEqual(self.mockedDM.cfe._manager, self.mockedManager)
        self.assertIsInstance(self.mockedDM.cfe, PDDLCostEvaluator)
        self.assertEqual(self.mockedDM.clients[0], uut._client_id)
        self.assertTrue(uut._active_manager)
        self.assertEqual(uut._delegation_manager, self.mockedDM)
        self.assertEqual(self.mockedDM.start_service_prefix, self.mockedManager.prefix)

    def test_manager_update_pursued_goals(self):
        """
        Tests manager client update actively pursued goals
        """

        uut = TestClientManager(manager=self.mockedManager)
        self.mockedDM = uut.used_delegation_manager
        uut.make_cost_computable()
        self.mockedDM.failed_goals = []

        goals = [MockedGoal(name="default")]
        test_goal = MockedGoal(name="test_goal")
        goals.append(test_goal)

        self.assertListEqual(self.mockedDM.failed_goals, [])
        uut.update_actively_pursued_goals(goals)
        self.assertListEqual(self.mockedDM.failed_goals, [])

        goals.remove(test_goal)
        uut.update_actively_pursued_goals(goals)
        self.assertListEqual(self.mockedDM.failed_goals, [test_goal.name])


if __name__ == '__main__':
    unittest.main()
