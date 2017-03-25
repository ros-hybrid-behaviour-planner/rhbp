#! /usr/bin/env python2
'''
Tests the goal implementations

Created on 25.03.2017

@author: rieger
'''
import time
import unittest

import rospy
import rostest
from behaviour_components.activators import Condition, ThresholdActivator
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.goals import OfflineGoal
from behaviour_components.managers import Manager
from behaviour_components.network_behavior import NetworkBehavior
from behaviour_components.pddl import Effect
from behaviour_components.sensors import SimpleTopicSensor
from std_msgs.msg import Int32

PKG = 'rhbp_core'

"""
System test for network behavior. Assumes, that a rosmaster is running
"""


class TopicIncreaserBehavior(BehaviourBase):
    """
    Behavior, which increases an int value
    """

    def __init__(self, effect, topic_name, name, **kwargs):
        super(TopicIncreaserBehavior, self).__init__(name, requires_execution_steps=True, **kwargs)
        self._correlations = [effect]
        self.__publisher = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.__next_value = 1
        rospy.sleep(1)

    def do_step(self):
        self.__publisher.publish(Int32(self.__next_value))
        self.__next_value += 1


class TestNetworkBehavior(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestNetworkBehavior, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TestNetworkBehavior' + str(time.time()).replace('.', '')
        rospy.init_node('NetworkBehaviorTestNode', log_level=rospy.DEBUG)

    def test_network_behavior(self):
        """
        Tests the case, that one network behavior is embedded into another network behavior.
        The goal requires to receive an int (3) in a topic.
        """

        method_prefix = self.__message_prefix + "TestNetworkBehavior"

        topic_name = method_prefix + '/Topic'
        sensor = SimpleTopicSensor(topic=topic_name, message_type=Int32, initial_value=False)
        condition = Condition(sensor, ThresholdActivator(thresholdValue=3))

        planner_prefix = method_prefix + "/Manager"
        # Disable planner, since the change from python to C
        #  disturbs the connection between the test process and the node process
        m = Manager(activationThreshold=7, planBias=0.0, prefix=planner_prefix)
        goal = OfflineGoal('CentralGoal')
        goal.add_condition(condition)
        m.add_goal(goal)

        pddl_function_name = condition.getFunctionNames()[0]
        effect = Effect(pddl_function_name, 1, sensorType=int)

        first_level_network = NetworkBehavior(name=method_prefix + '/FirstLevel', plannerPrefix=planner_prefix,
                                              effects=[(sensor, effect)], planBias=0.0)
        seccond_level_network = NetworkBehavior(name=method_prefix + '/SeccondLevel',
                                                plannerPrefix=first_level_network.get_manager_prefix(),
                                                effects=[(sensor, effect)], planBias=0.0)

        increaser_behavior = TopicIncreaserBehavior(effect=effect, topic_name=topic_name,
                                                    name=method_prefix + "TopicIncreaser",
                                                    plannerPrefix=seccond_level_network.get_manager_prefix())

        # activate the first_level_network, second_level_network and increaser_Behavior
        for x in range(0, 3, 1):
            self.assertFalse(first_level_network._isExecuting)
            m.step()
            rospy.sleep(0.1)
        self.assertTrue(first_level_network._isExecuting)

        for x in range(0, 3, 1):
            self.assertFalse(seccond_level_network._isExecuting)
            first_level_network.do_step()
            rospy.sleep(0.1)
        self.assertTrue(seccond_level_network._isExecuting)

        for x in range(0, 3, 1):
            self.assertFalse(increaser_behavior._isExecuting)
            seccond_level_network.do_step()
            rospy.sleep(0.1)
        self.assertTrue(increaser_behavior._isExecuting)

        # Satisfy goal
        for step in range(0, 3, 1):
            sensor.sync()
            self.assertEqual(step, sensor.value)
            seccond_level_network.do_step()
            rospy.sleep(0.1)

        goal.sync()
        self.assertTrue(goal.satisfied, 'Goal is not satisfied')


if __name__ == '__main__':
    rostest.rosrun(PKG, 'NetworkBehaviorTestNode', TestNetworkBehavior)
