"""
Unit tests for the Delegation- and DelegableBehaviour

Needs a running ROSCORE

@author: Mengers
"""

import unittest
import rospy
from behaviour_components.managers import Manager
from delegation_tests.test_utils import MockedClient
from decomposition_components.delegation_behaviour import DelegableBehaviour,\
    DelegationBehaviour
from behaviour_components.sensors import TopicSensor
from behaviour_components.conditions import Condition
from behaviour_components.activators import BooleanActivator
from behaviour_components.condition_elements import Effect
from std_msgs.msg import Bool


class TestDelegationBehaviour(DelegationBehaviour):
    """
    Overrides the Client with a mocked Client
    """

    def _create_delegation_client(self, checking_prefix):
        self._delegation_client = MockedClient()


class TestDelegableBehaviour(DelegableBehaviour):
    """
    Overrides the Client with a mocked Client and implements all abstract methods
    """

    # noinspection PyPep8Naming
    def __init__(self, name, plannerPrefix, work_cost,
                 satisfaction_threshold=1.0, delegation_depth_prefix=None,
                 **kwargs):
        super(TestDelegableBehaviour, self).__init__(name=name, plannerPrefix=plannerPrefix,
                                                     work_cost=work_cost,
                                                     satisfaction_threshold=satisfaction_threshold,
                                                     delegation_depth_prefix=delegation_depth_prefix,
                                                     **kwargs)

        self.last_work = None

    def _create_delegation_client(self, checking_prefix):
        self._delegation_client = MockedClient()

    def start_work(self):
        self.last_work = "start"

    def stop_work(self):
        self.last_work = "stop"

    def do_step_work(self):
        self.last_work = "do_step"


class DelegationBehaviourTest(unittest.TestCase):
    """
    Tests for the DelegationBehaviour
    """

    def setUp(self):
        rospy.init_node("TestNode")
        self.manager = Manager(prefix="test_manager")
        self.uut = TestDelegationBehaviour(name="test_behaviour", plannerPrefix="test_manager")
        self.sensor = TopicSensor(name="test_sensor", topic="/sensor_topic", message_type=Bool, initial_value=False)
        self.test_condition = Condition(self.sensor, BooleanActivator())
        self.effect = Effect(sensor_name="test_sensor", indicator=1.0)
        self.client = self.uut.delegation_client

    def tearDown(self):
        self.manager.unregister()
        self.uut.unregister()

    def test_start(self):
        self.uut.add_effect(self.effect)
        self.uut.add_condition_for_delegation(self.test_condition)
        self.uut.start()

        self.assertEqual("test_behaviourGoal", self.client.goal_name)
        self.assertEqual([self.test_condition], self.client.conditions)

    def test_do_step(self):
        self.uut.do_step()
        self.assertTrue(self.client.step_done)

    def test_stop(self):
        self.uut.stop()
        self.assertTrue(self.client.terminated)

    def test_success_func(self):
        self.uut.add_effect(self.effect)
        self.uut.add_condition_for_delegation(self.test_condition)
        self.uut.start()

        self.client.success_func()
        self.assertTrue(self.client.terminated)


class DelegableBehaviourTest(unittest.TestCase):
    """
    Tests for the DelegableBehaviour
    """

    def setUp(self):
        rospy.init_node("TestNode")
        self.manager = Manager(prefix="test_manager")
        self.uut = TestDelegableBehaviour(name="test_behaviour", plannerPrefix="test_manager", work_cost=5)
        self.sensor = TopicSensor(name="test_sensor", topic="/sensor_topic", message_type=Bool, initial_value=False)
        self.test_condition = Condition(self.sensor, BooleanActivator())
        self.effect = Effect(sensor_name="test_sensor", indicator=1.0)
        self.client = self.uut.delegation_client

    def tearDown(self):
        self.manager.unregister()
        self.uut.unregister()

    # ------ Not working myself ------

    def test_start(self):
        self.assertFalse(self.uut.currently_doing_work_locally)
        self.uut.add_effect(self.effect)
        self.uut.add_condition_for_delegation(self.test_condition)
        self.uut.start()

        self.assertEqual("test_behaviourGoal", self.client.goal_name)
        self.assertEqual([self.test_condition], self.client.conditions)
        self.assertFalse(self.uut.currently_doing_work_locally)
        self.assertIsNone(self.uut.last_work)

    def test_do_step(self):
        self.assertFalse(self.uut.currently_doing_work_locally)
        self.uut.do_step()
        self.assertIsNone(self.uut.last_work)
        self.assertTrue(self.client.step_done)

    def test_stop(self):
        self.assertFalse(self.uut.currently_doing_work_locally)
        self.uut.stop()
        self.assertIsNone(self.uut.last_work)
        self.assertTrue(self.client.terminated)

    def test_success_func(self):
        self.assertFalse(self.uut.currently_doing_work_locally)
        self.uut.add_effect(self.effect)
        self.uut.add_condition_for_delegation(self.test_condition)
        self.uut.start()
        self.assertIsNone(self.uut.last_work)

        self.client.success_func()
        self.assertTrue(self.client.terminated)

    # ------ Working myself (locally) ------

    def test_start_work_func(self):
        self.assertFalse(self.uut.currently_doing_work_locally)
        self.uut.add_effect(self.effect)
        self.uut.add_condition_for_delegation(self.test_condition)
        self.uut.start()
        self.assertIsNone(self.uut.last_work)
        self.assertFalse(self.uut.currently_doing_work_locally)

        self.client.start_work_func()
        self.assertTrue(self.uut.currently_doing_work_locally)
        self.assertEqual(self.uut.last_work, "start")

    def test_start_locally(self):
        self.assertFalse(self.uut.currently_doing_work_locally)
        self.uut.add_effect(self.effect)
        self.uut.add_condition_for_delegation(self.test_condition)
        self.client.toggle_manager_active()
        self.uut.start()

        self.assertTrue(self.uut.currently_doing_work_locally)
        self.assertEqual(self.uut.last_work, "start")

    def test_do_step_locally(self):
        self.assertFalse(self.uut.currently_doing_work_locally)
        self.uut.add_effect(self.effect)
        self.uut.add_condition_for_delegation(self.test_condition)
        self.uut.start()
        self.client.start_work_func()
        self.uut.do_step()
        self.assertTrue(self.uut.currently_doing_work_locally)
        self.assertEqual(self.uut.last_work, "do_step")

    def test_stop_locally(self):
        self.assertFalse(self.uut.currently_doing_work_locally)
        self.uut.add_effect(self.effect)
        self.uut.add_condition_for_delegation(self.test_condition)
        self.uut.start()
        self.client.start_work_func()
        self.assertTrue(self.uut.currently_doing_work_locally)
        self.uut.stop()
        self.assertFalse(self.uut.currently_doing_work_locally)
        self.assertEqual(self.uut.last_work, "stop")


if __name__ == '__main__':
    # Make sure a ROSCORE is running before starting
    unittest.main()
