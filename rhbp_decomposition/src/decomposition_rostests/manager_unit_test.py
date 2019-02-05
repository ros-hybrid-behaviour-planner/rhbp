#! /usr/bin/env python2
"""
Basic unit test for the rhbp_decomposition Manager

@author: Mengers
"""

from decomposition_components.managers import Manager
from delegation_module_tests.test_utils import MockedDelegationManager
import unittest
import rospy
import rostest


class ManagerUnitTest(unittest.TestCase):
    """
    Unit tests for the decomposition Manager
    """

    def setUp(self):
        rospy.init_node("TestNode")
        self.uut = Manager(prefix="test")
        # use a mocked DelegationManager
        self.uut.delegation_client.unregister()
        self.mockedDM = MockedDelegationManager()
        self.uut.delegation_client.register(self.mockedDM)

    def test_manager_participating(self):
        """
        Tests the participating functions
        """

        self.assertFalse(self.uut.participating_in_auctions)
        self.assertIsNone(self.mockedDM.cfe)
        self.uut.participate_in_auctions()
        self.assertTrue(self.uut.participating_in_auctions)
        self.assertIsNotNone(self.mockedDM.cfe)
        self.assertEqual(self.mockedDM.agent_name, "test")

        self.uut.stop_participating_in_auctions()
        self.assertFalse(self.uut.participating_in_auctions)
        self.assertIsNone(self.mockedDM.cfe)


if __name__ == '__main__':
    rostest.rosrun('rhbp_decomposition', 'test_manager_node', ManagerUnitTest)
    rospy.spin()
