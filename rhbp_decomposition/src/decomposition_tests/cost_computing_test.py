"""
Unit tests for the PDDLCostEvaluator

@author: Mengers
"""

import unittest
from decomposition_components.cost_computing import PDDLCostEvaluator
from delegation_components.delegation_errors import DelegationPlanningWarning
from decomposition_tests.test_utils import MockedManager


class CostComputingTest(unittest.TestCase):
    """
    Unit tests for the PDDLCostEvaluator
    """

    def setUp(self):
        self.manager = MockedManager()
        self.uut = PDDLCostEvaluator(manager=self.manager)
        self.uut.TASK_UTILIZATION_FACTOR = 1
        self.uut.WORKLOAD_PROPORTION_FACTOR = -0.5
        self.uut.ADDITIONAL_WORKLOAD_FACTOR = 1
        self.uut.ADDITIONAL_DELEGATION_FACTOR = 1
        self.uut.COOPERATION_AMOUNT_FACTOR = 1
        self.uut.CONTRACTOR_NUMBER_FACTOR = 0.5

    def test_basic(self):
        """
        Tests basic properties
        """

        self.assertEqual(self.uut.last_cost, -1)
        self.assertEqual(self.uut.last_possibility, False)

    def test_plan_steps_getting(self):
        """
        Tests the getting of plan steps
        """

        base, full, simple = self.uut._get_plan_steps(full_plan=self.manager.plan_with_additional_goal(0), simple_plan=self.manager.plan_this_single_goal(0))

        self.assertEqual(base, 2)
        self.assertEqual(full, 4)
        self.assertEqual(simple, 2)

    def test_number_of_delegations(self):
        """
        Tests the number of delegations in a plan
        """

        plan = dict()
        delegations = self.uut._determine_number_of_delegations(plan=plan)
        self.assertEqual(delegations, 0)

        plan = self.manager.plan
        delegations = self.uut._determine_number_of_delegations(plan=plan)
        self.assertEqual(delegations, 1)

    def test_compute_cost(self):
        """
        Tests the computing of cost
        """

        full_plan = self.manager.plan_with_additional_goal(0)
        simple_plan = self.manager.plan_this_single_goal(0)
        task_count = 1
        max_task_count = 4
        depth = 2
        max_depth = 8
        members = ["Member1", "Member2"]
        own_name = "Member1"

        cost = self.uut._compute_cost(full_plan=full_plan, simple_plan=simple_plan, task_count=task_count,
                                      max_task_count=max_task_count, depth=depth, max_depth=max_depth,
                                      members=members, own_name=own_name)

        self.assertLess(0, cost)
        self.assertEqual(cost, 10.546875)  # result for these parameters

    def test_compute_cost_and_possibility(self):
        """
        Tests full compute cost and possibility function
        """

        goal_representation = "doesnt matter for this test"
        current_task_count = 1
        max_task_count = 4
        current_depth = 2
        max_depth = 8
        members = ["Member1", "Member2"]
        own_name = "Member1"

        cost, possibility = self.uut.compute_cost_and_possibility(goal_representation=goal_representation, current_task_count=current_task_count,
                                                                  max_task_count=max_task_count, current_depth=current_depth,
                                                                  max_depth=max_depth, members=members, own_name=own_name)

        self.assertTrue(possibility)
        self.assertLess(0, cost)
        self.assertEqual(cost, 10.546875)  # result for these parameters
        self.assertEqual(self.uut.last_cost, cost)
        self.assertEqual(self.uut.last_possibility, possibility)

        self.manager.failing_plans = True
        cost, possibility = self.uut.compute_cost_and_possibility(goal_representation=goal_representation, current_task_count=current_task_count,
                                                                  max_task_count=max_task_count, current_depth=current_depth,
                                                                  max_depth=max_depth, members=members, own_name=own_name)

        self.assertFalse(possibility)
        self.assertEqual(cost, -1)
        self.assertEqual(self.uut.last_cost, cost)
        self.assertEqual(self.uut.last_possibility, possibility)

        self.manager.plan_exception = True
        self.assertRaises(DelegationPlanningWarning, self.uut.compute_cost_and_possibility,
                          goal_representation, current_task_count, max_task_count,
                          current_depth, max_depth, members, own_name)


if __name__ == '__main__':
    unittest.main()
