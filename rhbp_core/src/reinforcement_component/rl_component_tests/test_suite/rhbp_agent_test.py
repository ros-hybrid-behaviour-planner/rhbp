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
from behaviour_components.sensors import *
from behaviour_components.goals import OfflineGoal,GoalBase

from rcs_ros_bridge.msg import SimStart,  GenericAction,PlayMode, Goals,Flags,Lines

from threading import Event
import threading
from random import randint
import time
from diagnostic_msgs.msg import KeyValue
import gym
import re
import rospy
from reinforcement_component.rl_component_tests.test_suite.rhbp_agent_frozen_lake import FrozenLakeAgent
#from src.rhbp.rhbp_core.src.behaviour_components.sensors import Sensor


class RhbpAgentBaseOld():
    def __init__(self):
        rospy.logdebug("RhbpAgent::init")


        self.prefix = "test_agent"
        self._sim_started = False

        self.env = None


    def init_environment(self,name):

        self.env = gym.make(name)
        self.env.seed(0)

    def start_simulation(self):
        self.init_behaviors()

        state = self.env.reset()
        self.state_sensor.update(state)

    def init_behaviors(self):
        raise NotImplementedError()


if __name__ == '__main__':
    try:
        rospy.init_node('agent_node', anonymous=True)

        rhbp_agent = FrozenLakeAgent()
        rhbp_agent.start_simulation()
        print("init agent_node")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
