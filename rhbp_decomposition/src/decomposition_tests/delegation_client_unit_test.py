"""
Unit tests for the RHBPDelegationClient variations

@author: Mengers
"""

from decomposition_components.delegation_clients import RHBPDelegationClient, RHBPDelegableClient
from decomposition_components.manager_client import RHBPManagerDelegationClient
from decomposition_components.cost_computing import PDDLCostEvaluator
from delegation_tests.test_utils import MockedDelegationManager
from decomposition_tests.test_utils import MockedManager, FunctionPointerTester
import unittest


class RHBPClientsTest(unittest.TestCase):
    """
    Unit tests for the RHBPDelegationClient variations
    """

    def setUp(self):
        self.mockedDM = MockedDelegationManager()
        self.mockedManager = MockedManager()
        self.checking_prefix = "test_prefix"

    def test_delegate(self):
        """
        Tests delegate of the RHBPDelegationClient
        """

        uut = RHBPDelegationClient(checking_prefix=self.checking_prefix)

        goal_name = "test_goal"
        conditions = ["test_conditions"]
        threshold = 0.9

        self.assertRaises(RuntimeError, uut.delegate, goal_name, conditions, threshold)

        uut.register(delegation_manager=self.mockedDM)
        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold)

        self.assertEqual(delegation_id, 1)
        self.assertEqual(self.mockedDM.goal_wrapper._conditions, conditions)
        self.assertEqual(self.mockedDM.goal_wrapper._satisfaction_threshold, threshold)

    def test_manager_register_without_cost_eval(self):
        """
        Tests manager register, no cost eval
        """

        uut = RHBPManagerDelegationClient(manager=self.mockedManager)

        uut.register(delegation_manager=self.mockedDM, add_own_cost_evaluator=False)

        self.assertIsNone(self.mockedDM.cfe)
        self.assertEqual(self.mockedDM.clients[0], uut._client_id)
        self.assertTrue(uut._active_manager)
        self.assertEqual(uut._delegation_manager, self.mockedDM)

    def test_new_cost_evaluator(self):
        """
        Tests cost eval adding
        """

        uut = RHBPManagerDelegationClient(manager=self.mockedManager)

        cost_eval = uut.get_new_cost_evaluator()

        self.assertEqual(cost_eval._manager, self.mockedManager)
        self.assertIsInstance(cost_eval, PDDLCostEvaluator)

    def test_manager_register_with_cost_eval(self):
        """
        Tests manager register, cost eval added
        """

        uut = RHBPManagerDelegationClient(manager=self.mockedManager)

        uut.register(delegation_manager=self.mockedDM, add_own_cost_evaluator=True)

        self.assertEqual(self.mockedDM.cfe._manager, self.mockedManager)
        self.assertIsInstance(self.mockedDM.cfe, PDDLCostEvaluator)
        self.assertEqual(self.mockedDM.clients[0], uut._client_id)
        self.assertTrue(uut._active_manager)
        self.assertEqual(uut._delegation_manager, self.mockedDM)
        self.assertEqual(self.mockedDM.start_service_prefix, self.mockedManager.prefix)

    def test_delegable_delegate(self):
        """
        Tests delegable delegate
        """

        uut = RHBPDelegableClient(checking_prefix=self.checking_prefix)

        goal_name = "test_goal"
        conditions = ["test_conditions"]
        threshold = 0.9

        # base cases
        self.assertRaises(RuntimeError, uut.delegate, goal_name, conditions, threshold)

        uut.register(delegation_manager=self.mockedDM)
        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold)

        self.assertEqual(delegation_id, 1)
        self.assertEqual(self.mockedDM.goal_wrapper._conditions, conditions)
        self.assertEqual(self.mockedDM.goal_wrapper._satisfaction_threshold, threshold)

        # with own cost
        uut = RHBPDelegableClient(checking_prefix=self.checking_prefix)
        uut.register(delegation_manager=self.mockedDM)
        fpt = FunctionPointerTester()
        own_cost = 2.3

        self.assertRaises(RuntimeError, uut.delegate, goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold, own_cost=own_cost)

        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold, own_cost=own_cost, start_work_function=fpt.function)

        self.assertEqual(delegation_id, 1)
        self.assertEqual(self.mockedDM.goal_wrapper._conditions, conditions)
        self.assertEqual(self.mockedDM.goal_wrapper._satisfaction_threshold, threshold)
        self.assertEqual(uut._work_function_dictionary[delegation_id], fpt.function)

    def test_work_function_dict(self):
        """
        Tests work functions of DelegableClient
        :return:
        """
        uut = RHBPDelegableClient(checking_prefix=self.checking_prefix)
        uut.register(delegation_manager=self.mockedDM)
        fpt = FunctionPointerTester()
        own_cost = 2.3
        goal_name = "test_goal"
        conditions = ["test_conditions"]
        threshold = 0.9

        self.assertRaises(RuntimeError, uut.delegate, goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold, own_cost=own_cost)

        delegation_id = uut.delegate(goal_name=goal_name, conditions=conditions, satisfaction_threshold=threshold, own_cost=own_cost, start_work_function=fpt.function)

        uut.start_work_for_delegation(delegation_id=delegation_id)
        self.assertTrue(fpt.function_called)

        self.assertRaises(KeyError, uut.start_work_for_delegation, delegation_id + 1)

        uut.terminate_delegation(delegation_id=delegation_id)
        self.assertRaises(KeyError, uut.start_work_for_delegation, delegation_id)


if __name__ == '__main__':
    unittest.main()
