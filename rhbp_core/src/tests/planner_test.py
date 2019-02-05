'''
@author: hrabia
'''

import unittest

from behaviour_components.planner import MetricFF, MetricFFSearchMode


class PlannerTestSuite(unittest.TestCase):
    """Testing the planner class"""

    def setUp(self):
        # example domain
        self.domain_pddl = \
            "(define (domain UNNAMED) \n\
            (:requirements :strips :adl :equality :negation :conditional-effects :fluents) \n\
            (:predicates)\n\
            (:functions \n\
               (costs) \n\
               (temp_sensor) \n\
            ) \n\
            (:action increase_temp \n\
            :parameters () \
            :effect (and (increase (costs) 1.0) (increase (temp_sensor) 1)) \n\
            ) \n\
            (:action decrease_temp \n\
            :parameters () \n\
            :effect (and (increase (costs) 1.0) (decrease (temp_sensor) 1)) \n\
            )\n\
            )"

        # example problem
        self.problem_pddl = \
            "(define (problem problem-UNNAMED) \n\
             (:domain UNNAMED) \n\
                (:init \n\
                    ( = (temp_sensor) 10) \n\
                    ( = (costs) 0) \n\
                ) \n\
                (:goal ( > (temp_sensor) 15)) \n\
                (:metric minimize (costs)) \n\
            )"

    def test_astar_searches(self):
        """
        A star search planning is not able to solve the problem, unknown why
        """
        planner = MetricFF(search_mode=MetricFFSearchMode.A_STAR_EPSILON)
        self._check_plan_empty(planner)

        planner = MetricFF(search_mode=MetricFFSearchMode.A_STAR_WEIGHTED)
        self._check_plan_empty(planner)

    def test_planning(self):
        """
        Test all other and working search strategies
        """
        planner = MetricFF(search_mode=MetricFFSearchMode.EHC_H_A_STAR_EPSILON)
        self._check_plan(planner)

        planner = MetricFF(search_mode=MetricFFSearchMode.BFS)
        self._check_plan(planner)

        planner = MetricFF(search_mode=MetricFFSearchMode.BFS_H)
        self._check_plan(planner)

        planner = MetricFF(search_mode=MetricFFSearchMode.EHC_H_BFS)
        self._check_plan(planner)

    def _check_plan_empty(self, planner):
        plan = planner.plan(self.domain_pddl, self.problem_pddl)

        self.assertEquals(None, plan)

    def _check_plan(self, planner):
        plan = planner.plan(self.domain_pddl, self.problem_pddl)

        self.assertEquals(6, plan['cost'])
        self.assertTrue(plan['actions'])


if __name__ == '__main__':
    unittest.main()
