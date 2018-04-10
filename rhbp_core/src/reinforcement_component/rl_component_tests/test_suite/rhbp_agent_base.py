#!/usr/bin/env python2
# this is the main file of a rhbp agent instance
# it contains the agents variables, topics subscriber and publisher, as well as the regarding callback function
# Although the job_execution process is start in this file
from __future__ import division # force floating point division when using plain /
import gym
import re
import rospy

#from src.rhbp.rhbp_core.src.behaviour_components.sensors import Sensor
from behaviour_components.managers import Manager


class RhbpAgentBase(object):
    def __init__(self):

        self.prefix = "test_agent"
        self._sim_started = False

        self.env = None

        self.manager = None

    def set_manager(self):
        self.manager = Manager(prefix=self.prefix,
                               max_parallel_behaviours=1)  # ensure also max_parallel_behaviours during debugging

    def init_environment(self,name):

        self.env = gym.make(name)


    def start_simulation(self):
        self.init_behaviors()
        self.env.seed(0)
        state = self.env.reset()
        self.state_sensor.update(state)

        print("init env in state",state)

    def init_behaviors(self):
        raise NotImplementedError()