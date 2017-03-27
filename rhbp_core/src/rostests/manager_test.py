#! /usr/bin/env python2
'''
Tests manager implementation and integration

Created on 27.03.2017

@author: hrabia
'''
import time
import unittest

import rospy
import rostest
from behaviour_components.activators import Condition, BooleanActivator
from behaviour_components.goals import OfflineGoal, GoalBase
from behaviour_components.managers import Manager
from behaviour_components.sensors import SimpleTopicSensor
from std_msgs.msg import Bool

from tests.common import SetTrueBehavior

PKG = 'rhbp_core'

"""
System test for manager and its integration with other components
"""


class TestManager(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestManager, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TestManager_' + str(time.time()).replace('.', '')
        rospy.init_node('manager_test_node', log_level=rospy.DEBUG)
        # Disable planner, since the change from python to C
        #  disturbs the connection between the test process and the node process
        #rospy.set_param("~planBias", 0.0)

    def test_independent_behaviour(self):

        method_prefix = self.__message_prefix + "test_independent_behaviour"
        planner_prefix = method_prefix + "Manager"
        m = Manager(activationThreshold=7, prefix=planner_prefix)

        topic_name_1 = method_prefix + '/sensor_1'

        sensor = SimpleTopicSensor(topic=topic_name_1, message_type=Bool, initial_value=False)
        condition = Condition(sensor, BooleanActivator())

        condition_function_name = condition.getFunctionNames()[0]
        SetTrueBehavior(effect_name=condition_function_name, topic_name=topic_name_1,
                        name=method_prefix + "TopicIncreaser", plannerPrefix=planner_prefix)


        independent_behaviour = SetTrueBehavior(effect_name=condition_function_name, topic_name=topic_name_1,
                                                name=method_prefix + "TopicIncreaser2", plannerPrefix=planner_prefix, independentFromPlanner=True)


        goal = GoalBase(method_prefix + 'CentralGoal', plannerPrefix=planner_prefix)
        goal.addCondition(condition)

        for x in range(0, 3, 1):
            m.step()
            rospy.sleep(0.1)

        goal_proxy = m.goals[0]
        goal_proxy.sync()
        self.assertTrue(goal_proxy.satisfied, 'Goal is not satisfied')

        self.assertTrue(independent_behaviour._isExecuting, "Independent Behaviour not executed")



if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_goals_node', TestManager)
    rospy.spin()
