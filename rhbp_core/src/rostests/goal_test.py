#! /usr/bin/env python2
'''
Tests the goal implementations

Created on 15.12.2016

@author: rieger
'''
import time
import unittest

import rospy
import rostest
from std_msgs.msg import Bool
from behaviour_components.activators import Condition, BooleanActivator
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.goals import OfflineGoal, GoalBase
from behaviour_components.managers import Manager
from behaviour_components.pddl import Effect
from behaviour_components.sensors import SimpleTopicSensor

PKG = 'rhbp_core'

"""
System test for goals. Assumes, that a rosmaster is running
"""


class TopicIncreaserBehavior(BehaviourBase):
    """
    Behavior, which publishs True in the given topic
    """

    def __init__(self, effect_name, topic_name, name, **kwargs):
        super(TopicIncreaserBehavior, self).__init__(name, **kwargs)
        self._correlations = [Effect(effect_name, 1, sensorType=bool)]
        self.__publisher = rospy.Publisher(topic_name,Bool,queue_size=10)
        rospy.sleep(1)

    def start(self):
        self.__publisher.publish(True)
        self._isExecuting = False


class TestGoals(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestGoals, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TestGoals' + str(time.time()).replace('.', '')
        rospy.init_node('goal_test_node', log_level=rospy.DEBUG)

    def test_remote_goal(self):

        method_prefix = self.__message_prefix + "TestRemoteGoal"
        planner_prefix = method_prefix + "Manager"
        m = Manager(activationThreshold=7,planBias=0.0, prefix=planner_prefix)

        topic_name = method_prefix+ '/Topic'

        sensor = SimpleTopicSensor(topic=topic_name, message_type = Bool, initial_value = False)
        condition = Condition(sensor, BooleanActivator())

        pddl_function_name = condition.getFunctionNames()[0]
        TopicIncreaserBehavior(effect_name=pddl_function_name, topic_name=topic_name,
                                name=method_prefix + "TopicIncreaser", plannerPrefix=planner_prefix)
        goal = GoalBase(method_prefix + 'CentralGoal', plannerPrefix=planner_prefix)
        goal.addCondition(condition)

        for x in range(0, 3, 1):
            m.step()
            rospy.sleep(0.1)

        goal_proxy = m.goals[0]
        rospy.sleep(1)
        goal_proxy.sync()
        self.assertTrue(goal_proxy.satisfied, 'Goal is not satisfied')

    def test_offline_goal(self):

        method_prefix = self.__message_prefix + "TestOfflineGoal"
        planner_prefix = method_prefix + "/Manager"
        m = Manager(activationThreshold=7,planBias=0.0, prefix=planner_prefix)

        topic_name = method_prefix+ '/Topic'

        sensor = SimpleTopicSensor(topic=topic_name, message_type = Bool, initial_value = False)
        condition = Condition(sensor, BooleanActivator())

        pddl_function_name = condition.getFunctionNames()[0]
        TopicIncreaserBehavior(effect_name=pddl_function_name, topic_name=topic_name,
                                name=method_prefix + "TopicIncreaser", plannerPrefix=planner_prefix)
        goal = OfflineGoal('CentralGoal')
        goal.add_condition(condition)
        m.add_goal(goal)
        for x in range(0, 3, 1):
            m.step()
            rospy.sleep(0.1)

        rospy.sleep(1)
        goal.sync()
        self.assertTrue(goal.satisfied, 'Goal is not satisfied')


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_goals_node', TestGoals)
