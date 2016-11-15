'''
@author: hrabia
'''

import unittest

import behaviour_components.activators as activators

class LinearActivatorTestSuite(unittest.TestCase):
    """Testing activation and wish calculation of LinearActivator"""

    def test_increasing_range_med_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=0, fullActivationValue=10, minActivation=0,
                                               maxActivation=1)

        activation = activator.computeActivation(5)
        self.assertEqual(activation, 0.5)

        wish = activator.getSensorWish(5)
        self.assertEqual(wish, 0.5)

    def test_increasing_range_small_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=0, fullActivationValue=10, minActivation=0,
                                               maxActivation=1)

        activation = activator.computeActivation(1)
        self.assertEqual(activation, 0.1)

        wish = activator.getSensorWish(1)
        self.assertEqual(wish, 0.9)

    def test_increasing_range_large_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=0, fullActivationValue=10, minActivation=0,
                                               maxActivation=1)

        activation = activator.computeActivation(9)
        self.assertEqual(activation, 0.9)

        wish = activator.getSensorWish(9)
        self.assertEqual(wish, 0.1)

    def test_decreasing_range_med_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=10, fullActivationValue=0, minActivation=0,
                                               maxActivation=1)

        activation = activator.computeActivation(5)
        self.assertEqual(activation, 0.5)

        wish = activator.getSensorWish(5)
        self.assertEqual(wish, -0.5)

    def test_decreasing_range_small_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=10, fullActivationValue=0, minActivation=0,
                                               maxActivation=1)

        activation = activator.computeActivation(1)
        self.assertEqual(activation, 0.9)

        wish = activator.getSensorWish(1)
        self.assertEqual(wish, -0.1)


    def test_decreasing_range_large_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=10, fullActivationValue=0, minActivation=0,
                                               maxActivation=1)

        activation = activator.computeActivation(9)
        self.assertEqual(activation, 0.1)

        wish = activator.getSensorWish(9)
        self.assertEqual(wish, -0.9)

if __name__ == '__main__':
    unittest.main()