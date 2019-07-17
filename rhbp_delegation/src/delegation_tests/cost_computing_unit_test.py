"""
Unit tests for the PDDLCostEvaluator

@author: Mengers
"""

import unittest
import math
from decomposition_components.cost_computing import PDDLCostEvaluator
from delegation_components.delegation_errors import DelegationPlanningWarning
from delegation_components.cost_evaluators import CostParameters
from test_utils import MockedManager


class CostComputingTest(unittest.TestCase):
    """
    Unit tests for the PDDLCostEvaluator
    """

    def setUp(self):
        self.manager = MockedManager()
        self.uut = PDDLCostEvaluator(manager=self.manager)
        # setting for these parameters
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

        self.assertTrue(math.isnan(self.uut.last_cost))
        self.assertEqual(self.uut.last_possibility, False)

    def test_plan_steps_getting(self):
        """
        Tests the getting of plan steps
        """

        base, full, simple = self.uut._get_plan_steps(full_plan=self.manager.plan_with_additional_goal(0),
                                                      simple_plan=self.manager.plan_this_single_goal(0))

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

    def test_plan_and_extraxt_paramters(self):
        """
        Tests the plan_and_extract_parameters function
        """

        goal_rep = "test goal"
        task_count = 1
        max_task_count = 4
        depth = 2
        max_depth = 8
        members = ["Member1", "Member2"]
        own_name = "Member1"

        resulting_paramters = self.uut._plan_and_extract_parameters(goal_representation=goal_rep, own_name=own_name,
                                                                    members=members, depth=depth, max_depth=max_depth,
                                                                    task_count=task_count, max_task_count=max_task_count)

        # resulting parameter test
        self.assertEqual(resulting_paramters.name, own_name)
        self.assertEqual(resulting_paramters.base_steps, 2)
        self.assertEqual(resulting_paramters.full_steps, 4)
        self.assertEqual(resulting_paramters.simple_steps, 2)
        self.assertEqual(resulting_paramters.depth, depth)
        self.assertEqual(resulting_paramters.max_depth, max_depth)
        self.assertEqual(resulting_paramters.new_contractor, 0)
        self.assertEqual(resulting_paramters.contractor_count, len(set(members)))
        self.assertEqual(resulting_paramters.new_delegations, 1)
        self.assertEqual(resulting_paramters.num_delegations, 1)
        self.assertEqual(resulting_paramters.task_count, task_count)
        self.assertEqual(resulting_paramters.max_task_count, max_task_count)

    def test_cost_evaluate(self):
        """
        Tests the computing of cost
        """

        test_parameters = CostParameters(base_steps=2, full_steps=4, simple_steps=2, depth=2, max_depth=8,
                                         new_contractor=0, contractor_count=2, new_delegations=1,
                                         num_delegations=1, task_count=1, max_task_count=4)

        cost = self.uut.cost_evaluate(parameters=test_parameters)

        self.assertLess(0, cost)
        self.assertEqual(cost, 10.546875)  # result for these exact parameters

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

        cost, possibility = self.uut.compute_cost_and_possibility(goal_representation=goal_representation,
                                                                  current_task_count=current_task_count,
                                                                  max_task_count=max_task_count,
                                                                  current_depth=current_depth,
                                                                  max_depth=max_depth, members=members,
                                                                  own_name=own_name)

        self.assertTrue(possibility)
        self.assertLess(0, cost)
        self.assertEqual(cost, 10.546875)  # result for these parameters
        self.assertEqual(self.uut.last_cost, cost)
        self.assertEqual(self.uut.last_possibility, possibility)

        self.manager.failing_plans = True
        self.assertRaises(DelegationPlanningWarning, self.uut.compute_cost_and_possibility,
                          goal_representation=goal_representation, current_task_count=current_task_count,
                          max_task_count=max_task_count, current_depth=current_depth, max_depth=max_depth,
                          members=members, own_name=own_name)

        self.assertTrue(math.isnan(self.uut.last_cost))
        self.assertFalse(self.uut.last_possibility)

        self.manager.plan_exception = True
        self.assertRaises(DelegationPlanningWarning, self.uut.compute_cost_and_possibility,
                          goal_representation, current_task_count, max_task_count,
                          current_depth, max_depth, members, own_name)


if __name__ == '__main__':
    unittest.main()
