'''
@author: hrabia
'''

import unittest

import rospy

from behaviour_components.pddl import create_valid_pddl_name, PDDL, mergeStatePDDL


class PDDLTestSuite(unittest.TestCase):
    """Testing pddl"""

    def test_pddl_name(self):
        """Testing valid name generation for the PDDL language"""
        self.assertEqual('MyNetworkManager', create_valid_pddl_name('MyNetwork/Manager'))

    def test_empty(self):
        """
        test PDDL.empty property
        """
        empty_pddl = PDDL()
        non_empty_pddl = PDDL(statement="(:action test_action\n:parameters ()\n", functions="costs")

        self.assertTrue(empty_pddl.empty, "Empty PDDL is not empty")
        self.assertFalse(non_empty_pddl.empty, "not empty PDDL is empty")

    def test_merge(self):

        function_name = "one"

        function_one = "( = (" + function_name + ") {0} )".format(1)
        function_two = "( = (" + function_name + ") {0} )".format(2)
        one = PDDL(statement=function_one, functions=function_name, time_stamp=rospy.Time(secs=10))

        two = PDDL(statement=function_two, functions=function_name, time_stamp=rospy.Time(secs=11))

        merged = mergeStatePDDL(one, two)

        self.assertIsNotNone(merged)

        self.assertEquals(function_two, merged.statement)

        two.time_stamp = rospy.Time(secs=9)  # check again with two becoming older

        merged = mergeStatePDDL(one, two)

        self.assertIsNotNone(merged)

        self.assertEquals(function_one, merged.statement)


if __name__ == '__main__':
    unittest.main()
