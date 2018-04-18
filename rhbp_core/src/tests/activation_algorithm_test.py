'''
@author: hrabia
'''

import unittest

from behaviour_components.activation_algorithm import ActivationAlgorithmFactory,BaseActivationAlgorithm

from behaviour_components.managers import Manager


class ActivationAlgorithmTestSuite(unittest.TestCase):

    def setUp(self):

        self.manager = Manager()

    def tearDown(self):

        self.manager.unregister()

    def test_factory(self):

        algo = ActivationAlgorithmFactory.create_algorithm("default", self.manager)

        self.assertNotEqual(algo, None, 'Algorithm is none')

        self.assertTrue(isinstance(algo, BaseActivationAlgorithm), "Not the correct algorithm")


if __name__ == '__main__':
    unittest.main()
