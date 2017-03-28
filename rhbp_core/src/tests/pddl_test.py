'''
@author: hrabia
'''

import unittest

from behaviour_components.pddl import create_valid_pddl_name, PDDL

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
        non_empty_pddl = PDDL(statement =  "(:action test_action\n:parameters ()\n", functions = "costs")

        self.assertTrue(empty_pddl.empty, "Empty PDDL is not empty")
        self.assertFalse(non_empty_pddl.empty, "not empty PDDL is empty")


if __name__ == '__main__':
    unittest.main()
