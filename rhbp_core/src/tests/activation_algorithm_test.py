'''
@author: hrabia
'''

import unittest

import collections

from behaviour_components.activation_algorithm import ActivationAlgorithmFactory,BaseActivationAlgorithm

from behaviour_components.managers import Manager


from mock import patch
from rospy.rostime import Time


class ActivationAlgorithmTestSuite(unittest.TestCase):

    def setUp(self):
        self._rospy_patcher = patch('rospy.Time.now')
        self._rospy_patcher.start()
        self._rospy_patcher.return_value = Time(0, 0)

        DynamicReconfigureServer = collections.namedtuple('DynamicReconfigureServer', 'ns')

        Manager.dynamic_reconfigure_server = DynamicReconfigureServer(ns="UNUSED")
        self.manager = Manager(activationThreshold=7, createLogFiles=False, max_parallel_behaviours=1)

    def tearDown(self):

        self.manager.unregister()
        self._rospy_patcher.stop()

    def test_factory(self):

        algo = ActivationAlgorithmFactory.create_algorithm("default", self.manager)

        self.assertNotEqual(algo, None, 'Algorithm is none')

        self.assertTrue(isinstance(algo, BaseActivationAlgorithm), "Not the correct algorithm")


if __name__ == '__main__':
    unittest.main()
