'''
@author: hrabia
'''

import unittest

import collections

from behaviour_components.activation_algorithm import ActivationAlgorithmFactory, BaseActivationAlgorithm

from behaviour_components.managers import Manager


from mock import patch, MagicMock
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

    def test_goal_priority(self):
        """
        Testing priority dependent goal activation calculation
        """

        algo = BaseActivationAlgorithm(self.manager, extensive_logging=True)
        algo._apply_goal_priority_weights = True

        class BehaviourMock(object):

            def __init__(self, name, priority):
                self.priority = priority
                self.name =name

            def __str__(self):
                return self.name

        b1 = BehaviourMock("A", 1)
        b2 = BehaviourMock("B", 2)

        self.manager._operational_goals.append(b1)
        self.manager._operational_goals.append(b2)
        algo.step_preparation()
        self.assertEqual(algo._goal_priority_weights[b1], 1)
        self.assertEqual(algo._goal_priority_weights[b2], 2)

        # now test normalisation
        b1.priority = 2
        b2.priority = 4
        algo.step_preparation()
        self.assertEqual(algo._goal_priority_weights[b1], 1)
        self.assertEqual(algo._goal_priority_weights[b2], 2)

        b3 = BehaviourMock("C", 3)
        self.manager._operational_goals.append(b3)
        algo.step_preparation()
        self.assertEqual(algo._goal_priority_weights[b3], 1.5)


if __name__ == '__main__':
    unittest.main()
