import unittest
import rospy

from behaviour_components.managers import Manager
from behaviour_components.sensors import TopicSensor
from behaviour_components.conditions import Condition
from behaviour_components.activators import BooleanActivator
from std_msgs.msg import Bool

from decomposition_components.goal_wrapper import DecompositionGoal


class DecompositionGoalTest(unittest.TestCase):

    def setUp(self):
        rospy.init_node("TestNode")
        sensor = TopicSensor(name="test_sensor", topic="sensor_topic", message_type=Bool, initial_value=False)
        self.conditions = [Condition(sensor, BooleanActivator())]
        self.manager_name = "Test_manager"
        self.manager = Manager(prefix=self.manager_name)
        self.goal_name = "test_goal"
        self.satisfaction_threshold = 1.0

    def tearDown(self):
        self.manager.unregister()

    def test_check_alive(self):
        uut = DecompositionGoal(name=self.goal_name, plannerPrefix=self.manager_name, conditions=self.conditions, satisfaction_threshold=self.satisfaction_threshold)
        self.assertTrue(uut.check_if_alive())
        self.manager.unregister()
        self.assertFalse(uut.check_if_alive())


if __name__ == '__main__':
    # Make sure a ROSCORE is running before starting
    unittest.main()