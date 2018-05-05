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

from reinforcement_component.rl_component_tests.taxi.dqn.test_base import BaseTestSuite
from rhbp_core.msg import InputState
from rhbp_core.srv import GetActivation

PKG = 'knowledge_base'

"""
System test for knowledge base fact cache.
"""


class TaxiTestSuiteDQN(BaseTestSuite):
    def __init__(self,algorithm=0, *args, **kwargs):
        super(TaxiTestSuiteDQN, self).__init__(algorithm,*args, **kwargs)

    def do_step(self,input):
        # e-greedy random selection without restriction

        # get the best action
        best_action = self.get_best_action(input, self.last_r, self.last_action)
        i = self.counter
        self.counter += 1

        # choose randomly best action
        random_value = numpy.random.rand(1)

        if random_value < self.epsilon or self.counter - 1 < self.pre_train:
            best_action = numpy.random.randint(self.num_outputs)
        # execute best action
        s1, r, d, _ = self.env.step(best_action)

        # reduce epsilon
        if self.epsilon > self.endE and self.counter-1 > self.pre_train:
            self.epsilon -= self.stepDrop

        self.last_action = best_action

        return s1,r,d


if __name__ == '__main__':
    #rostest.rosrun(PKG, 'update_handler_test_node', UpdateHandlerTestSuite)
    fl_test = TaxiTestSuiteDQN()
    fl_test.start_env(num_prints=100,threshold=-500,random_seed=0,should_print=True)
    fl_test.start_env(num_prints=100, threshold=-400, random_seed=1,should_print=True)