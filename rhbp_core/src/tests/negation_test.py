'''
@author: hrabia
'''

import unittest

import behaviour_components.activators as activators
import behaviour_components.conditions as conditions
import behaviour_components.sensors as sensors

from mock import patch
from rospy.rostime import Time


class NegationTestSuite(unittest.TestCase):
    """Testing Negation Condition"""

    def setUp(self):
        self._rospy_patcher = patch('rospy.Time.now')
        self._rospy_patcher.start()
        self._rospy_patcher.return_value = Time(0, 0)

    def tearDown(self):
        self._rospy_patcher.stop()

    def test_negation_increasing_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=0, fullActivationValue=10, minActivation=0,
                                               maxActivation=1)

        mock_sensor = sensors.Sensor(initial_value=3)

        condition = conditions.Condition(sensor=mock_sensor, activator=activator)

        neg_condition = conditions.Negation(condition)

        neg_condition.updateComputation()

        con_satisfaction = condition.satisfaction
        neg_con_satisfaction = neg_condition.satisfaction

        self.assertAlmostEqual(con_satisfaction, 0.3, places=3)
        self.assertAlmostEqual(neg_con_satisfaction, 0.7, places=3)

        con_wish = condition.getWishes()[0]
        neg_con_wish = neg_condition.getWishes()[0]

        self.assertAlmostEqual(con_wish.indicator, 0.7, places=3)
        self.assertAlmostEqual(neg_con_wish.indicator, -0.3, places=3)

    def test_negation_decreasing_activation(self):
        activator = activators.LinearActivator(zeroActivationValue=10, fullActivationValue=0, minActivation=0,
                                               maxActivation=1)

        mock_sensor = sensors.Sensor(initial_value=3)

        condition = conditions.Condition(sensor=mock_sensor, activator=activator)

        neg_condition = conditions.Negation(condition)

        neg_condition.updateComputation()

        con_satisfaction = condition.satisfaction
        neg_con_satisfaction = neg_condition.satisfaction

        self.assertAlmostEqual(con_satisfaction, 0.7, places=3)
        self.assertAlmostEqual(neg_con_satisfaction, 0.3, places=3)

        con_wish = condition.getWishes()[0]
        neg_con_wish = neg_condition.getWishes()[0]

        self.assertAlmostEqual(con_wish.indicator, -0.3, places=3)
        self.assertAlmostEqual(neg_con_wish.indicator, 0.7, places=3)

    def test_negation_increasing_activation_out_of_range(self):
        activator = activators.LinearActivator(zeroActivationValue=0, fullActivationValue=10, minActivation=0,
                                               maxActivation=1)

        mock_sensor = sensors.Sensor(initial_value=12)

        condition = conditions.Condition(sensor=mock_sensor, activator=activator)

        neg_condition = conditions.Negation(condition)

        neg_condition.updateComputation()

        con_satisfaction = condition.satisfaction
        neg_con_satisfaction = neg_condition.satisfaction

        self.assertAlmostEqual(con_satisfaction, 1.0, places=3)
        self.assertAlmostEqual(neg_con_satisfaction, 0.0, places=3)

        con_wish = condition.getWishes()[0]
        neg_con_wish = neg_condition.getWishes()[0]

        self.assertAlmostEqual(con_wish.indicator, 0.0, places=3)
        self.assertAlmostEqual(neg_con_wish.indicator, -1.0, places=3)

    def test_negation_decreasing_activation_out_of_range(self):
        activator = activators.LinearActivator(zeroActivationValue=10, fullActivationValue=5, minActivation=0,
                                               maxActivation=1)

        mock_sensor = sensors.Sensor(initial_value=3)

        condition = conditions.Condition(sensor=mock_sensor, activator=activator)

        neg_condition = conditions.Negation(condition)

        neg_condition.updateComputation()

        con_satisfaction = condition.satisfaction
        neg_con_satisfaction = neg_condition.satisfaction

        self.assertAlmostEqual(con_satisfaction, 1.0, places=3)
        self.assertAlmostEqual(neg_con_satisfaction, 0.0, places=3)

        con_wish = condition.getWishes()[0]
        neg_con_wish = neg_condition.getWishes()[0]

        self.assertAlmostEqual(con_wish.indicator, 0.0, places=3)
        self.assertAlmostEqual(neg_con_wish.indicator, 1.0, places=3)

    def test_negation_boolean_true(self):
        mock_sensor = sensors.Sensor(initial_value=True)

        activator = activators.BooleanActivator(desiredValue=False)

        condition = conditions.Condition(sensor=mock_sensor, activator=activator)

        neg_condition = conditions.Negation(condition)

        neg_condition.updateComputation()

        con_satisfaction = condition.satisfaction
        neg_con_satisfaction = neg_condition.satisfaction

        self.assertAlmostEqual(con_satisfaction, 0.0, places=3)
        self.assertAlmostEqual(neg_con_satisfaction, 1.0, places=3)

        con_wish = condition.getWishes()[0]
        neg_con_wish = neg_condition.getWishes()[0]

        self.assertAlmostEqual(con_wish.indicator, -1.0, places=3)
        self.assertAlmostEqual(neg_con_wish.indicator, 0.0, places=3)

    def test_negation_boolean_false(self):
        mock_sensor = sensors.Sensor(initial_value=False)

        activator = activators.BooleanActivator(desiredValue=False)

        condition = conditions.Condition(sensor=mock_sensor, activator=activator)

        neg_condition = conditions.Negation(condition)

        neg_condition.updateComputation()

        con_satisfaction = condition.satisfaction
        neg_con_satisfaction = neg_condition.satisfaction

        self.assertAlmostEqual(con_satisfaction, 1.0, places=3)
        self.assertAlmostEqual(neg_con_satisfaction, 0.0, places=3)

        con_wish = condition.getWishes()[0]
        neg_con_wish = neg_condition.getWishes()[0]

        self.assertAlmostEqual(con_wish.indicator, 0.0, places=3)
        self.assertAlmostEqual(neg_con_wish.indicator, 1.0, places=3)


if __name__ == '__main__':
    unittest.main()