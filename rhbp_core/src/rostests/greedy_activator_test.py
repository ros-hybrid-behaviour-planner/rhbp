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
from behaviour_components.activators import GreedyActivator
from behaviour_components.conditions import Condition
from behaviour_components.goals import OfflineGoal
from behaviour_components.managers import Manager
from behaviour_components.sensors import TopicSensor
from std_msgs.msg import Int32

from tests.common import IncreaserBehavior

PKG = 'rhbp_core'

"""
System test for the GreedyActivator
"""


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

        sensor = TopicSensor(topic=topic_name, name='IncreaseTopicTestSensor', message_type=Int32,
                             initial_value=0)
        activator = GreedyActivator()
        condition = Condition(sensor=sensor, activator=activator)

        IncreaserBehavior(topic_name=topic_name, effect_name=sensor.name,
                          createLogFiles=True, planner_prefix=planner_prefix)
        goal = OfflineGoal(name=self.__message_prefix + 'CentralGoal', planner_prefix=planner_prefix)
        goal.add_condition(condition)
        m.add_goal(goal)

        number_of_steps = 15
        for x in range(0, number_of_steps + 1 , 1):
            m.step()
            rospy.sleep(0.1)

        # it takes 2 steps until the activation has increased
        expected_behaviour_steps = number_of_steps - 2
        self.assertEquals(expected_behaviour_steps, sensor.latestValue)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'greedy_activator_test_node', TestGreedyActivator)
