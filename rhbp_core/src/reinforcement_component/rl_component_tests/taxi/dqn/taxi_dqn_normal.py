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


class TaxiTestNormal(BaseTestSuite):
    def __init__(self,algorithm=0, *args, **kwargs):
        super(TaxiTestNormal, self).__init__(algorithm,*args, **kwargs)

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


class TaxiTestConditions(BaseTestSuite):
    def __init__(self,algorithm=0, *args, **kwargs):
        super(TaxiTestConditions, self).__init__(algorithm,*args, **kwargs)

    def send_invalid_action_to_rl(self, input_state, reward, last_action_index):

        input_state_msg = InputState()
        input_state_msg.input_state = input_state.tolist()[0]
        input_state_msg.num_outputs = self.num_outputs
        input_state_msg.num_inputs = self.num_inputs
        input_state_msg.reward = reward
        input_state_msg.last_action = last_action_index
        input_state_msg.resulting_state = self.resulting_state.tolist()[0]

        self.rl_component.save_request(input_state_msg)

    def do_step(self,input):
        # e-greedy random selection without restriction
        s = numpy.argmax(input)
        best_action = self.get_best_action(input, self.last_r, self.last_action)
        i = self.counter
        self.counter += 1
        # choose randomly best action
        random_value = numpy.random.rand(1)
        if random_value < self.epsilon or self.counter - 1 < self.pre_train:
            best_action = numpy.random.randint(6)
        # execute best action
        while not self.is_action_valid(s, best_action):
            minus_value = -100

            self.send_invalid_action_to_rl(self.get_array(s), minus_value, best_action)

            self.activation_rl[best_action] = minus_value
            best_action = numpy.argmax(self.activation_rl)

        s1, r, d, _ = self.env.step(best_action)

        # reduce epsilon
        if self.epsilon > self.endE and self.counter - 1 > self.pre_train:
            self.epsilon -= self.stepDrop

        self.last_action = best_action


        return s1,r,d



class TaxiTestDecoded(TaxiTestNormal):
    """
    uses normal exploration strategy without condition (like. but decodes state into four variables with on_hot
    """
    def __init__(self,algorithm=0, *args, **kwargs):
        super(TaxiTestDecoded, self).__init__(algorithm,*args, **kwargs)

        self.num_inputs = 19

    def get_array(self,s):
        row,col,passenger,dest=self.decode(s)

        array = self.one_hot(row, 5)
        array = numpy.concatenate( (array,self.one_hot(col, 5)),axis=1)
        array = numpy.concatenate((array, self.one_hot(passenger, 5)),axis=1)
        array = numpy.concatenate((array, self.one_hot(dest, 4)),axis=1)
        return array



if __name__ == '__main__':
    #rostest.rosrun(PKG, 'update_handler_test_node', UpdateHandlerTestSuite)
    fl_test = TaxiTestNormal()
    fl_test.start_env(num_prints=100,threshold=-500,random_seed=0,should_print=True)
    fl_test.start_env(num_prints=100, threshold=-400, random_seed=1,should_print=True)