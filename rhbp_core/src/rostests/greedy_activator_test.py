#! /usr/bin/env python2
'''
Tests the greedy activator

Created on 15.12.2016

@author: rieger
'''

import time
import unittest

import rospy
import rostest
from behaviour_components.activators import Condition, GreedyActivator
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.goals import OfflineGoal
from behaviour_components.managers import Manager
from behaviour_components.pddl import Effect
from behaviour_components.sensors import SimpleTopicSensor
from std_msgs.msg import Int32

PKG = 'rhbp_core'

"""
System test for the GreedyActivator
"""


class IncreaserBehavior(BehaviourBase):
    """
    Behavior, which increases an int value
    """

    def __init__(self, topic_name, effect_name, name='increaserAdderBehaviour',
                 **kwargs):
        super(IncreaserBehavior, self).__init__(name, **kwargs)
        self._correlations = [Effect(effect_name, 1, sensorType=int)]
        self.__publisher = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.__next_value = 1
        rospy.sleep(1)

    def start(self):
        self.__publisher.publish(Int32(self.__next_value))
        self.__next_value += 1
        self._isExecuting = False


class TestGreedyActivator(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestGreedyActivator, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TestGreedyActivator' + str(time.time()).replace('.', '')
        rospy.init_node('TestGreedyActivatorTestNode', log_level=rospy.DEBUG)
        # Disable planner, since the change from python to C
        #  disturbs the connection between the test process and the node process
        rospy.set_param("~planBias", 0.0)

    def test_activator(self):
        method_prefix = self.__message_prefix + "TestOfflineGoal"
        planner_prefix = method_prefix + "Manager"
        m = Manager(activationThreshold=7, prefix=planner_prefix)

        topic_name = 'IncreaseTopicTest/Topic'

        sensor = SimpleTopicSensor(topic=topic_name, name='IncreaseTopicTestSensor', message_type=Int32,
                                   initial_value=0)
        activator = GreedyActivator()
        condition = Condition(sensor=sensor, activator=activator)

        pddl_function_name = condition.getFunctionNames()[0]

        IncreaserBehavior(topic_name=topic_name, effect_name=pddl_function_name,
                          createLogFiles=True, plannerPrefix=planner_prefix)
        goal = OfflineGoal(name=self.__message_prefix + 'CentralGoal')
        goal.add_condition(condition)
        m.add_goal(goal)

        for x in range(0, 16, 1):
            m.step()
            rospy.sleep(0.1)

        self.assertEquals(7, sensor.latestValue)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'greedy_activator_test_node', TestGreedyActivator)
