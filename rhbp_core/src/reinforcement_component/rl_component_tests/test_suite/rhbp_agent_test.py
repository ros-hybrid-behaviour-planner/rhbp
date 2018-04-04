#!/usr/bin/env python2
# this is the main file of a rhbp agent instance
# it contains the agents variables, topics subscriber and publisher, as well as the regarding callback function
# Although the job_execution process is start in this file
from __future__ import division # force floating point division when using plain /
from agent_modules_rcs_contest import *
from behaviour_components.activators import BooleanActivator, StringActivator, ThresholdActivator, GreedyActivator, \
    LinearActivator, EqualActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition, MultiSensorCondition, Disjunction
from behaviour_components.managers import Manager
from behaviour_components.sensors import *
from behaviour_components.network_behavior import NetworkBehaviour
from behaviour_components.goals import OfflineGoal,GoalBase
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from rcs_ros_bridge.msg import SimStart,  GenericAction,PlayMode, Goals,Flags,Lines

from threading import Event
import threading
from random import randint
import time
from diagnostic_msgs.msg import KeyValue
import gym
import re
import rospy

#from src.rhbp.rhbp_core.src.behaviour_components.sensors import Sensor


class RhbpAgentBase:
    def __init__(self):
        rospy.logdebug("RhbpAgent::init")


        self._agent_name = rospy.get_param('~agent_name', 'agentA1')  # default for debugging 'TUBDAI1'

        #self._agent_topic_prefix = get_bridge_topic_prefix(agent_name=self._agent_name)



        #self._manager = Manager(prefix=self._agent_name, max_parallel_behaviours=1)
        # ensure also max_parallel_behaviours during debugging


        # a client for the Knowledge base for writing and reading
        #self.client = KnowledgeBaseClient(knowledge_base_name=KNOWLEDGE_BASE_NAME)

        self.prefix = "test_agent"
        self._sim_started = False

        self.env = None


        'FrozenLake-v0'
    def init_environment(self,name):

        self.env = gym.make(name)


    def start_simulation(self):
        self.init_behaviors()

        state = self.env.reset()
        self.state_sensor.update(state)



    def init_behaviors(self):
        """
        here we could also evaluate the msg_old in order to initialize depending on the role etc.
        :param msg:  the message
        :type msg: SimStart
        """
        self.environment_name = 'FrozenLake-v0'
        #self.environment_name = 'Taxi-v2'
        rhbp_agent.init_environment(self.environment_name)



        self.state_sensor = Sensor(name="StateSensor")

        #self.state_sensor2 = Sensor(name="StateSensor2")
        #self.state_sensor2.update(4)
        action_one_behavior = MakeActionBehavior(plannerPrefix=self.prefix,name="ActionOne",
                                                 action_index=0,state_sensor=self.state_sensor,environment=self.env)

        action_two_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionTwo",
                                                 action_index=1,state_sensor=self.state_sensor, environment=self.env)

        action_three_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionThree",
                                                 action_index=2,state_sensor=self.state_sensor, environment=self.env)

        action_four_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionFour",
                                                 action_index=3,state_sensor=self.state_sensor, environment=self.env)

        #action_five_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionFive",
        #                                         action_index=0,state_sensor=self.state_sensor, environment=self.env)
        #action_six_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionSix",
        #                                         action_index=0,state_sensor=self.state_sensor, environment=self.env)

        #is_in_goal_state = Condition(self.state_sensor,EqualActivator(desiredValue=15)) #TODO only for frozenlake
        is_in_goal_state = Condition(self.state_sensor, ThresholdActivator(thresholdValue=15))  # TODO only for frozenlake
        #start_state_cond = Condition(self.state_sensor, EqualActivator(desiredValue=0))  # TODO only for frozenlake

        #hole_1 = Condition(self.state_sensor, EqualActivator(desiredValue=5))
        #hole_2 = Condition(self.state_sensor, EqualActivator(desiredValue=7))
        #hole_3 = Condition(self.state_sensor, EqualActivator(desiredValue=11))
        #hole_4 = Condition(self.state_sensor, EqualActivator(desiredValue=12))
        #is_in_hole = Disjunction(hole_1,hole_2,hole_3,hole_4)
        #is_in_goal_state2 = Condition(self.state_sensor2, BooleanActivator(desiredValue=14))
        #action_four_behavior.addPrecondition(is_in_goal_state2)

        the_goal = GoalBase(name="goal_the", permanent=True,
                                          conditions=[is_in_goal_state], priority=1,
                                          plannerPrefix=self.prefix)

        #no_hole = GoalBase(name="no_hole_goal", permanent=True,
        #                    conditions=[Negation(start_state_cond)], priority=1,
        #                    plannerPrefix=self.prefix)
        direct_to_goal_effect = Effect(sensor_name=is_in_goal_state.getFunctionNames()[0], indicator=1.0,
                               sensor_type=float)#

        action_two_behavior.correlations.append(direct_to_goal_effect)
        action_one_behavior.correlations.append(direct_to_goal_effect)
        action_three_behavior.correlations.append(direct_to_goal_effect)
        action_four_behavior.correlations.append(direct_to_goal_effect)

if __name__ == '__main__':
    try:
        rospy.init_node('agent_node', anonymous=True)
        rhbp_agent = RhbpAgentBase()

        #rhbp_agent.init_environment()
        #rhbp_agent.init_behaviors()
        rhbp_agent.start_simulation()
        print("init agent_node")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
