#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: rieger, hrabia
'''
import time
import unittest

import rospy
import rostest
import roslaunch
from reinforcement_component.rl_component import RLComponent
import gym
import numpy
import pandas
import matplotlib.pyplot as plt

from rhbp_core.msg import InputState
from rhbp_core.srv import GetActivation

PKG = 'knowledge_base'

"""
System test for knowledge base fact cache.
"""


class FrozenLakeTestSuite():
    def __init__(self, *args, **kwargs):
        #super(UpdateHandlerTestSuite, self).__init__(*args, **kwargs)
        self.set_up_environment()
        self.rewards = 0
        self.cycles = 0

        self.rewards_last = 0
        self.cycles_last = 0
        self.weights = []
    def set_up_environment(self):
        self.env = gym.make('FrozenLake-v0')
    def get_array(self,s):
        return numpy.identity(16)[s:s + 1]

    def start_env(self):


        for i in range(1,4):
            s = self.env.reset()
            d = 0
            while not d:
                self.env.render()
                s1,d = self.make_cycle(s,i)
                s = s1
            if i%200 == 1:
                print(self.rewards,i,(self.rewards/i)*100,self.rewards_last/200)
                self.rewards_last = 0
                self.cycles_last = 0

        return

    def make_cycle(self,s,i):

        best_action = 1
        if s == 1:
            best_action = 2
        elif s == 3:
            best_action = 0
        elif s == 8:
            best_action = 2
        elif s == 13:
            best_action = 2
        elif s == 14:
            best_action = 2

        #print(input,best_action)
        s1, r, d, _ = self.env.step(best_action)


        #print(s1,r,d)
        self.rewards+=r
        self.cycles+=1

        self.rewards_last += r
        self.cycles_last += 1
        #print(s,s1,best_action)
        return s1, d



if __name__ == '__main__':
    #rostest.rosrun(PKG, 'update_handler_test_node', UpdateHandlerTestSuite)
    fl_test = FrozenLakeTestSuite()
    fl_test.start_env()