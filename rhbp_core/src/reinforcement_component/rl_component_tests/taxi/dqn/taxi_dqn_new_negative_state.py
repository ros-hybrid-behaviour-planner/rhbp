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
#from reinforcement_component.rl_component import RLComponent
import gym
import numpy
import pandas
import matplotlib.pyplot as plt
from test_base import BaseTestSuite
from rhbp_core.msg import InputState
from rhbp_core.srv import GetActivation

#from rhbp_core.src.reinforcement_component.rl_component_tests.taxi.dqn.test_base import BaseTestSuite

PKG = 'knowledge_base'

"""
System test for knowledge base fact cache.
"""


class TaxiTestConditionsNew(BaseTestSuite):
    def __init__(self,algorithm=0, *args, **kwargs):
        super(TaxiTestConditionsNew, self).__init__(algorithm,*args, **kwargs)

    def fetchActivation(self, msg,negative_states):
        '''
        This method fetches the status from the actual behaviour node via GetStatus service call
        '''
        self.SERVICE_TIMEOUT = 1
        self._justFinished_state = False
        try:
            d = 3
        except rospy.ROSException:
            return
        try:
            activation_result = self.rl_component.get_activation_state_test(msg,negative_states)
            self.activation_rl = activation_result.activations
        except rospy.ServiceException as e:
            print(e.message)

    def get_negative_states(self,s,input):
        num_actions = 6
        negative_states = []
        for action_index in range(num_actions):
            # if random chosen number is not executalbe, give minus reward and sent it to rl component
            if not self.is_action_valid(s,action_index):
                negative_state = InputState()
                negative_state.input_state = input.tolist()[0]
                negative_state.num_outputs = self.num_outputs
                negative_state.num_inputs = self.num_inputs
                negative_state.reward = -100
                negative_state.last_action = action_index
                negative_states.append(negative_state)
        return negative_states

    def do_step(self,input):
        # e-greedy random selection without restriction
        s = numpy.argmax(input)
        negative_states = self.get_negative_states(s,input)
        best_action = self.get_best_action(input, self.last_r, self.last_action,negative_states)
        i = self.counter
        self.counter += 1
        # choose randomly best action
        random_value = numpy.random.rand(1)
        if random_value < self.epsilon or self.counter - 1 < self.pre_train:
            best_action = numpy.random.randint(6)
        # execute best action
        while not self.is_action_valid(s, best_action):
            minus_value = -100

            #self.send_invalid_action_to_rl(self.get_array(s), minus_value, best_action)

            self.activation_rl[best_action] = minus_value
            best_action = numpy.argmax(self.activation_rl)

        s1, r, d, _ = self.env.step(best_action)

        # reduce epsilon
        if self.epsilon > self.endE and self.counter - 1 > self.pre_train:
            self.epsilon -= self.stepDrop

        self.last_action = best_action


        return s1,r,d

    def get_best_action(self, input_state, reward, last_action_index,negative):
        input_state_msg = InputState()
        input_state_msg.input_state = input_state.tolist()[0]
        input_state_msg.num_outputs = self.num_outputs
        input_state_msg.num_inputs = self.num_inputs
        input_state_msg.reward = reward
        input_state_msg.last_action = last_action_index
        input_state_msg.resulting_state = self.resulting_state.tolist()[0]

        self.fetchActivation(input_state_msg,negative)
        return numpy.argmax(self.activation_rl)


class TaxiTestAllConditionsNew(TaxiTestConditionsNew):
    def __init__(self,algorithm=0, *args, **kwargs):
        super(TaxiTestAllConditionsNew, self).__init__(algorithm,*args, **kwargs)

    def is_action_valid(self, s, a):

        locs = [(0, 0), (0, 4), (4, 0), (4, 3)]
        row, col, passenger, dest = self.decode(s)
        if a == 4:
            if passenger == 4:
                return False
            if locs[passenger][0] == row and locs[passenger][1] == col:
                return True
            return False
        if a == 5:
            if not passenger == 4:
                return False
            if locs[dest][0] == row and locs[dest][1] == col:
                return True
            return False
        return True


