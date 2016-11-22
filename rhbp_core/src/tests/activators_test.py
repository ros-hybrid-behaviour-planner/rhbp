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
        sensor_value = 5

        activation = activator.computeActivation(sensor_value)
        self.assertEqual(activation, 0.5)

        wish = activator.getSensorWish(sensor_value)
        self.assertEqual(wish, 0.5)

    def test_increasing_range_small_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=0, fullActivationValue=10, minActivation=0,
                                               maxActivation=1)

        sensor_value = 1

        activation = activator.computeActivation(sensor_value)
        self.assertEqual(activation, 0.1)

        wish = activator.getSensorWish(sensor_value)
        self.assertEqual(wish, 0.9)

    def test_increasing_range_large_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=0, fullActivationValue=10, minActivation=0,
                                               maxActivation=1)

        sensor_value = 9

        activation = activator.computeActivation(sensor_value)
        self.assertEqual(activation, 0.9)

        wish = activator.getSensorWish(sensor_value)
        self.assertEqual(wish, 0.1)

    def test_decreasing_range_med_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=10, fullActivationValue=0, minActivation=0,
                                               maxActivation=1)

        sensor_value = 5

        activation = activator.computeActivation(sensor_value)
        self.assertEqual(activation, 0.5)

        wish = activator.getSensorWish(sensor_value)
        self.assertEqual(wish, -0.5)

    def test_decreasing_range_large_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=10, fullActivationValue=0, minActivation=0,
                                               maxActivation=1)

        sensor_value = 1

        activation = activator.computeActivation(sensor_value)
        self.assertEqual(activation, 0.9)

        wish = activator.getSensorWish(sensor_value)
        self.assertEqual(wish, -0.1)


    def test_decreasing_range_small_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=10, fullActivationValue=0, minActivation=0,
                                               maxActivation=1)

        sensor_value = 9

        activation = activator.computeActivation(sensor_value)
        self.assertEqual(activation, 0.1)

        wish = activator.getSensorWish(sensor_value)
        self.assertEqual(wish, -0.9)


    def test_decreasing_range_small_activation2(self):
        activator = activators.LinearActivator(zeroActivationValue=10, fullActivationValue=5, minActivation=0,
                                               maxActivation=1)

        sensor_value = 9

        activation = activator.computeActivation(sensor_value)
        self.assertEqual(activation, 0.2)

        wish = activator.getSensorWish(sensor_value)
        self.assertEqual(wish, -0.8)


    def test_decreasing_range_out_of_max_activation_range(self):
        activator = activators.LinearActivator(zeroActivationValue=10, fullActivationValue=5, minActivation=0,
                                               maxActivation=1)
        sensor_value = 2

        activation = activator.computeActivation(sensor_value)
        self.assertEqual(activation, 1)

        wish = activator.getSensorWish(sensor_value)
        self.assertEqual(wish, 0)

    def test_decreasing_range_out_of_min_activation_range_(self):
        activator = activators.LinearActivator(zeroActivationValue=10, fullActivationValue=5, minActivation=0,
                                               maxActivation=1)
        sensor_value = 12

        activation = activator.computeActivation(sensor_value)
        self.assertEqual(activation, 0)

        wish = activator.getSensorWish(sensor_value)
        self.assertEqual(wish, -1)

    def test_increasing_range_out_of_min_activation_range(self):
        activator = activators.LinearActivator(zeroActivationValue=5, fullActivationValue=10, minActivation=0,
                                               maxActivation=1)
        sensor_value = 2

        activation = activator.computeActivation(sensor_value)
        self.assertEqual(activation, 0)

        wish = activator.getSensorWish(sensor_value)
        self.assertEqual(wish, 1)

    def test_increasing_range_out_of_max_activation_range_(self):
        activator = activators.LinearActivator(zeroActivationValue=5, fullActivationValue=10, minActivation=0,
                                               maxActivation=1)
        sensor_value = 12

        activation = activator.computeActivation(sensor_value)
        self.assertEqual(activation, 1)

        wish = activator.getSensorWish(sensor_value)
        self.assertEqual(wish, 0)


if __name__ == '__main__':
    unittest.main()