#! /usr/bin/env python2

import unittest
import rospy
import rostest

from behaviour_components.managers import Manager
from decomposition_components.cost_computing import PDDLCostEvaluator
from basic_sceanrio import BasicCookingRobot       # TODO change this

PKG = "rhbp_decomposition"

# TODO this test needs to be updated


class CostComputingTest(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(CostComputingTest, self).__init__(*args, **kwargs)
        rospy.init_node("TestNode1")

    def test_basic(self):
        prefix = "test_manager"

        m = Manager(prefix=prefix)

        bcr = BasicCookingRobot(planner_prefix=prefix)

        m.add_goal(bcr.get_test_goal())

        for i in range(7):
            m.step()
            rospy.sleep(1)

        goal = bcr.get_goal()

        cost_computer = PDDLCostEvaluator(manager=m.plan_with_additional_goal)

        cost, possible = cost_computer.compute_cost_and_possibility(goal.fetchPDDL()[0].statement)

        print ("Possibility:"+str(possible)+" , Cost:"+str(cost))

        self.assertTrue(possible)
        self.assertLess(0, cost)


if __name__ == '__main__':
    # Run this only with the launch file or with a passive node running
    rostest.rosrun(PKG, 'CostComputingTest', CostComputingTest)
