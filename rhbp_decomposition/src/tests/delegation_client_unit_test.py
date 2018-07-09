
from decomposition_components.delegation_clients import RHBPDelegationClient, RHBPManagerDelegationClient
from decomposition_components.cost_computing import PDDLCostEvaluator
from delegation_tests.test_utils import MockedDelegationManager
import unittest


class MockedManager(object):

    def __init__(self):
        self._prefix = "test_prefix"

    def plan_with_additional_goal(self):
        return self._prefix


class RHBPClientsTest(unittest.TestCase):

    def setUp(self):
        self.mockedDM = MockedDelegationManager()
        self.mockedManager = MockedManager()

    def test_delegate(self):
        uut = RHBPDelegationClient()

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
        uut = RHBPManagerDelegationClient(manager=self.mockedManager)

        uut.register(delegation_manager=self.mockedDM, add_own_cost_evaluator=False)

        self.assertIsNone(self.mockedDM.cfe)
        self.assertEqual(self.mockedDM.clients[0], uut._client_id)
        self.assertTrue(uut._active_manager)
        self.assertEqual(uut._delegation_manager, self.mockedDM)

    def test_new_cost_evaluator(self):
        uut = RHBPManagerDelegationClient(manager=self.mockedManager)

        cost_eval = uut.get_new_cost_evaluator()

        self.assertEqual(cost_eval._planning_function, self.mockedManager.plan_with_additional_goal)
        self.assertIsInstance(cost_eval, PDDLCostEvaluator)

    def test_manager_register_with_cost_eval(self):
        uut = RHBPManagerDelegationClient(manager=self.mockedManager)

        uut.register(delegation_manager=self.mockedDM, add_own_cost_evaluator=True)

        self.assertEqual(self.mockedDM.cfe._planning_function, self.mockedManager.plan_with_additional_goal)
        self.assertIsInstance(self.mockedDM.cfe, PDDLCostEvaluator)
        self.assertEqual(self.mockedDM.clients[0], uut._client_id)
        self.assertTrue(uut._active_manager)
        self.assertEqual(uut._delegation_manager, self.mockedDM)


if __name__ == '__main__':
    unittest.main()
