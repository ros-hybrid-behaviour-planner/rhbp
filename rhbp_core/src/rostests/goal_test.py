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
from behaviour_components.activators import Condition, BooleanActivator
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.goals import OfflineGoal, GoalBase
from behaviour_components.managers import Manager
from behaviour_components.pddl import Effect
from behaviour_components.sensors import KnowledgeSensor
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.msg import Push

PKG = 'rhbp_core'

"""
System test for goals. Assumes, that a rosmaster and the knowledge base is running
"""


class KnowledgeAdderBehaviour(BehaviourBase):
    """
    Behavior, which adds a fact at execution to knowledge base
    """

    def __init__(self, knwoledge_sensor_name, fact, name='knowledgeAdderBehaviour', **kwargs):
        super(KnowledgeAdderBehaviour, self).__init__(name, **kwargs)
        self._correlations = [Effect(knwoledge_sensor_name, 1, sensorType=bool)]
        self.__publisher = rospy.Publisher(KnowledgeBase.DEFAULT_NAME + KnowledgeBase.PUSH_TOPIC_NAME_POSTFIX, Push,
                                           queue_size=10)
        self.__fact = fact
        rospy.sleep(1)

    def start(self):
        self.__publisher.publish(Push(content=self.__fact))
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
        m = Manager(activationThreshold=7, prefix=planner_prefix)

        fact = (self.__message_prefix, 'test_offline_goal', 'Fact')

        sensor_name = 'SimpleKnowledgeSensor'
        sensor = KnowledgeSensor(fact, sensor_name=sensor_name)
        condition = Condition(sensor, BooleanActivator())

        pddl_function_name = condition.getFunctionNames()[0]
        KnowledgeAdderBehaviour(knwoledge_sensor_name=pddl_function_name, fact=fact,
                                name=method_prefix + "KnowledgeAdder", plannerPrefix=planner_prefix)
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
        planner_prefix = method_prefix + "Manager"
        m = Manager(activationThreshold=7, prefix=planner_prefix)

        fact = (self.__message_prefix, 'test_offline_goal', 'Fact')

        sensor_name = 'SimpleKnowledgeSensor'
        sensor = KnowledgeSensor(fact, sensor_name=sensor_name)
        condition = Condition(sensor, BooleanActivator())

        pddl_function_name = condition.getFunctionNames()[0]
        KnowledgeAdderBehaviour(knwoledge_sensor_name=pddl_function_name, fact=fact,
                                name=method_prefix + "KnowledgeAdder", plannerPrefix=planner_prefix)
        goal = OfflineGoal('CentralGoal')
        goal.addCondition(condition)
        m.add_goal(goal)

        for x in range(0, 3, 1):
            m.step()
            rospy.sleep(0.1)

        rospy.sleep(1)
        goal.sync()
        self.assertTrue(goal.satisfied, 'Goal is not satisfied')


if __name__ == '__main__':
    rostest.rosrun(PKG, 'goal_test_node', TestGoals)
