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
        self.resulting_state=numpy.array([[]])
        self.rewards = 0
        self.cycles = 0

        self.rewards_last = 0
        self.cycles_last = 0
        self.weights = []
        self.last_r=0
        self.last_action=0
        self.rl_address="test_agent"
        numpy.random.seed(0)
        self.set_up_environment()
        self.new_count = 0
        self.steps = 0
    def set_up_environment(self):
        self.rl_component=RLComponent(self.rl_address)
        self.env = gym.make('FrozenLake-v0')
        self.env.seed(0)
        self.num_inputs = 16
        self.num_outputs = 4

    def get_array(self,s):
        return numpy.identity(self.num_inputs)[s:s + 1]

    def start_env(self):

        num_prints = 100

        for i in range(1,3500):
            s = self.env.reset()
            d = 0
            while not d:

                s1,d = self.make_cycle(s,i)
                print(i, numpy.round(self.activation_rl,5))
                s = s1
            if i%num_prints == 1:
                print(self.rewards,i,(self.rewards/i)*100,self.rewards_last/num_prints)
                self.rewards_last = 0
                self.cycles_last = 0
                weight=self.rl_component.model.get_value_of_weight()

                self.weights.append(weight)

        for i in range(self.num_inputs):
            input = self.get_array(i)
            activations = self.rl_component.model.feed_forward(input)

            best_action = numpy.argmax(activations)
            #print(i,best_action)

    def print_weights(self,state):
        #print(self.weights[0][0].reshape([1,4]))
        column_list = ["Left","Down","Right","Up"]
        df = pandas.DataFrame([],columns=column_list)
        for w in self.weights:
            weight = w[state].reshape([1,self.num_outputs])
            df1 = pandas.DataFrame(weight,columns=column_list)
            #print(df1)
            df = df.append(df1,ignore_index=True)
        #print(df)

        ax = df[column_list].plot()
        #plt.show()

        plt.show()

    def make_cycle(self,s,i):

        input = self.get_array(s)

        #self.rl_component.check_if_model_is_valid(self.num_inputs, self.num_outputs)

        #activations = self.rl_component.model.feed_forward(input)

        #best_action = numpy.argmax(activations)
        best_action=self.get_best_action(input,self.last_r,self.last_action)

        # choose randomly best action
        #if i < 1000:
        #print(i)
        #i = self.steps
        self.steps +=1
        self.epsilon = 1. / ((i / 50) + 10)
        if numpy.random.rand(1)<self.epsilon:
            #best_action= self.env.action_space.sample()
            best_action  = numpy.random.randint(4)
            #print(i,"random action",best_action)
        #execute best action
        s1, r, d, _ = self.env.step(best_action)

        self.last_r = r*10
        self.last_action=best_action

        #print(s1,r,d)
        self.rewards+=r
        self.cycles+=1

        self.rewards_last += r
        self.cycles_last += 1

        self.resulting_state = self.get_array(s1)
        #print(s,s1,best_action)
        #if (s == s1):
            #print"same state" , s,s1,best_action
        #    r = -0.5
        #if r == 0:
        #    r = 10
        #if (s1 == 0):
        #    self.last_r=-10
        if (d == True) and not self.last_r == 10:
            self.last_r = -10
        self.last_r=self.last_r/10
        #if (d == True) and  r == 1:
        #    r = 10
        #if self.last_r == 0:
        #    self.last_r = -0.1
        output=self.get_array(s1)
        #print("-----------------")
        #print((self.new_count,numpy.argmax(input),numpy.argmax(output),best_action,r))
        self.new_count+=1
        #self.rl_component.reward_list.append((input,output,best_action,r))

        #self.rl_component.update_model()
        return s1, d

    """
 
, array([0.00162, 0.00616, 0.00071, 0.00031]))
(1, array([0.00049, 0.00094, 0.00426, 0.00285]))
(1, array([0.00415, 0.00456, 0.0013 , 0.00174]))
(1, array([0.00049, 0.00094, 0.00431, 0.00285]))
(0.0, 1, 0.0, 0.0)
(2, array([0.00162, 0.00577, 0.00071, 0.00031]))
(2, array([0.00769, 0.00069, 0.00797, 0.00122]))
(2, array([0.00162, 0.00619, 0.00071, 0.00031]))
(2, array([0.00162, 0.00618, 0.00071, 0.00031]))
(2, array([0.00769, 0.00069, 0.00761, 0.00122]))
(2, array([0.00162, 0.00647, 0.00071, 0.00031]))

(1, array([0.00162, 0.00616, 0.00071, 0.00031]))
(1, array([0.00769, 0.00069, 0.00797, 0.00122]))
(1, array([0.00162, 0.0065 , 0.00071, 0.00031]))
(1, array([0.00769, 0.00069, 0.00767, 0.00122]))
(1, array([0.00162, 0.00673, 0.00071, 0.00031]))
(1, array([0.00748, 0.00069, 0.00767, 0.00122]))
(2, array([0.00162, 0.0069 , 0.00071, 0.00031]))
(2, array([0.00162, 0.00688, 0.00071, 0.00031]))
(2, array([0.00049, 0.00094, 0.00426, 0.00285]))
(2, array([0.00049, 0.00094, 0.00425, 0.00285]))
(3, array([0.00162, 0.00635, 0.00071, 0.00031]))
(3, array([ 0.00049,  0.00094, -0.19535,  0.00285]))
(3, array([0.00162, 0.00564, 0.00071, 0.00031]))
(3, array([0.00162, 0.00563, 0.00071, 0.00031]))
(3, array([ 0.00049,  0.00094, -0.19535,  0.0034 ]))
(3, array([0.00162, 0.00518, 0.00071, 0.00031]))


    """
    def get_best_action(self, input_state, reward, last_action_index):
        input_state_msg = InputState()
        input_state_msg.input_state = input_state.tolist()[0]
        input_state_msg.num_outputs = self.num_outputs
        input_state_msg.num_inputs = self.num_inputs
        input_state_msg.reward = reward
        input_state_msg.last_action = last_action_index
        input_state_msg.resulting_state = self.resulting_state.tolist()[0]

        self.fetchActivation(input_state_msg)
        return numpy.argmax(self.activation_rl)

    def fetchActivation(self, msg):
        '''
        This method fetches the status from the actual behaviour node via GetStatus service call
        '''
        self.SERVICE_TIMEOUT=1
        self._justFinished_state = False
        try:
            d=3
            #rospy.wait_for_service(self.rl_address + 'GetActivation', timeout=self.SERVICE_TIMEOUT)
        except rospy.ROSException:
            return
        try:
            #getActivationRequest = rospy.ServiceProxy(self.rl_address + 'GetActivation', GetActivation)
            #activation_result = getActivationRequest(msg)
            activation_result = self.rl_component.get_activation_state_test(msg)
            self.activation_rl = activation_result.activations

            # if self._name != condition_state.name:
            #    rhbplog.logerr("%s fetched a status message from a different behaviour: %s. This cannot happen!", self._name, status.name)
            # rhbplog.logdebug("%s reports the following status:\nactivation %s\ncorrelations %s\nprecondition satisfaction %s\n ready threshold %s\nwishes %s\nactive %s\npriority %d\ninterruptable %s",
            #                 self._name, self._activationFromPreconditions, self._correlations, self._preconditionSatisfaction,
            #                 self._readyThreshold, self._wishes, self._active, self._priority, self._interruptable)
        except rospy.ServiceException as e:
            print(e.message)

if __name__ == '__main__':
    #rostest.rosrun(PKG, 'update_handler_test_node', UpdateHandlerTestSuite)
    fl_test = FrozenLakeTestSuite()
    fl_test.start_env()