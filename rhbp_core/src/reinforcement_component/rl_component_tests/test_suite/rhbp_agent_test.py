#!/usr/bin/env python2
# this is the main file of a rhbp agent instance
# it contains the agents variables, topics subscriber and publisher, as well as the regarding callback function
# Although the job_execution process is start in this file
from __future__ import division # force floating point division when using plain /
from agent_modules_tests import *
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
from reinforcement_component.rl_component_tests.test_suite.rhbp_agent_frozen_lake_manager import FrozenLakeAgentManager
from reinforcement_component.rl_component_tests.test_suite.rhbp_agent_taxi import TaxiAgent
from reinforcement_component.rl_component_tests.test_suite.rhbp_agent_taxi_with_rhbp import TaxiAgentRhbp
from reinforcement_component.rl_component_tests.test_suite.rhbp_agent_taxi_with_rhbp_undecoded import \
    TaxiAgentRhbpUndecoded


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
        sim = rospy.get_param('~sim', '0')  # default for debugging 'agentA1'
        if sim == 0:
            print("start envionment frozen lake")
            rhbp_agent = FrozenLakeAgent()
        elif sim == 1:
            print("start envionment frozen lake with included manager")
            rhbp_agent = FrozenLakeAgentManager()
            rhbp_agent.set_manager()

        elif sim == 2:
            print("start envionment taxi ")
            rhbp_agent = TaxiAgent()
        elif sim == 3:
            print("start envionment taxi with rhbp")
            rhbp_agent = TaxiAgentRhbp()
        elif sim == 4:
            print("start envionment taxi with rhbp undecoded")
            rhbp_agent = TaxiAgentRhbpUndecoded()
        #rhbp_agent = TaxiAgent()
        rhbp_agent.start_simulation()

        print("init agent_node")

        #if sim == 1:
        #    t = threading.Thread(target=rhbp_agent.make_step,args=())
        #    t.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
