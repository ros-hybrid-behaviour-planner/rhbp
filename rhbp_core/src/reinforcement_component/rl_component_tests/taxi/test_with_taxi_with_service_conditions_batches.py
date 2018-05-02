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
from reinforcement_component.reinforcement_learning_constants import RLConstants

class TaxiTestSuite():
    def __init__(self, *args, **kwargs):
        #super(UpdateHandlerTestSuite, self).__init__(*args, **kwargs)
        self.resulting_state=numpy.array([[]])
        self.rewards = 0
        self.cycles = 0
        numpy.random.seed(0)
        self.rewards_last = 0
        self.cycles_last = 0
        self.weights = []
        self.last_r=0
        self.last_action=0
        self.rl_address="test_agent"
        self.set_up_environment()
        self.counter = 0
        self.rewards_all = 0
        self.counter_last=0
    def set_up_environment(self):
        self.rl_component=RLComponent(self.rl_address)
        self.env = gym.make('Taxi-v2')

        self.env.seed(0)
        self.num_inputs = 500
        self.num_outputs = 6

    def get_array(self,s):
        return numpy.identity(self.num_inputs)[s:s + 1]

    def decode(self, i):
        out = []
        out.append(i % 4)  # row
        i = i // 4
        out.append(i % 5)  # col
        i = i // 5
        out.append(i % 5)  # passloc
        i = i // 5
        out.append(i)  # destination
        assert 0 <= i < 5
        return reversed(out)

    def is_action_valid(self,s,a):
        locs = [(0, 0), (0, 4), (4, 0), (4, 3)]
        row,col,passenger,dest=self.decode(s)
        if a==4:
            if passenger ==4:
                return False
            if locs[passenger][0] == row and locs[passenger][1] == col:
                return True
            return False
        if a==5:
            if not passenger==4:
                return False
            if locs[dest][0] == row and locs[dest][1] == col:
                return True
            return False
        return True

    def start_env(self):

        num_prints = 10

        for i in range(1,13500):
            s = self.env.reset()
            d = 0
            while not d:
                s1,d = self.make_cycle(s,i)
                s = s1
            self.cycles_last+=1
            if self.cycles_last == num_prints:
                print(self.counter,self.rewards_all/float(self.counter_last),self.rewards,i,(self.rewards/i)*100,self.rewards_last/float(num_prints))
                if self.rewards_last/float(num_prints)>0:
                    print("finished ")
                    return
                self.rewards_last = 0
                self.cycles_last = 0
                self.counter_last=0
                self.rewards_all=0


        for i in range(self.num_inputs):
            input = self.get_array(i)
            activations = self.rl_component.model.feed_forward(input)

            best_action = numpy.argmax(activations)
            print(i,best_action)

    def print_weights(self,state):
        #print(self.weights[0][0].reshape([1,4]))
        column_list = ["Left","Down","Right","Up"]
        df = pandas.DataFrame([],columns=column_list)
        for w in self.weights:
            weight = w[state].reshape([1,self.num_outputs])
            df1 = pandas.DataFrame(weight,columns=column_list)
            df = df.append(df1,ignore_index=True)
        ax = df[column_list].plot()

        plt.show()

    def make_cycle(self,s,i):

        input = self.get_array(s)

        #self.rl_component.check_if_model_is_valid(self.num_inputs, self.num_outputs)

        #activations = self.rl_component.model.feed_forward(input)

        #best_action = numpy.argmax(activations)
        best_action=self.get_best_action(input,self.last_r,self.last_action)
        i = self.counter
        self.counter +=1
        # choose randomly best action
        self.epsilon = 1. / ((i / 50) + 10)
        random_value = numpy.random.rand(1)
        #print(i,self.counter, self.epsilon, random_value, random_value < self.epsilon)
        if random_value < self.epsilon:
            best_action=numpy.random.randint(self.num_outputs)
        #execute best action
        # find invalid actions
        while not self.is_action_valid(s,best_action):
            minus_value=-100
            self.send_invalid_action_to_rl(self.get_array(s), minus_value, best_action)
            self.activation_rl[best_action]= minus_value
            best_action = numpy.argmax(self.activation_rl)

        s1, r, d, _ = self.env.step(best_action)
        self.last_r = r
        self.last_action=best_action
        self.rewards_all += r
        #print(s1,r,d)
        self.rewards+=r
        self.cycles+=1
        self.counter_last+=1
        self.rewards_last += r

        self.resulting_state = self.get_array(s1)

        output=self.get_array(s1)

        return s1, d

    def send_invalid_action_to_rl(self,input_state,reward,last_action_index):
        input_state_msg = InputState()
        input_state_msg.input_state = input_state.tolist()[0]
        input_state_msg.num_outputs = self.num_outputs
        input_state_msg.num_inputs = self.num_inputs
        input_state_msg.reward = reward
        input_state_msg.last_action = last_action_index
        input_state_msg.resulting_state = self.resulting_state.tolist()[0]

        self.rl_component.save_request(input_state_msg,False)

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
    fl_test = TaxiTestSuite()
    fl_test.start_env()