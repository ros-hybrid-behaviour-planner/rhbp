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
        self.rl_component=RLComponent()
        self.env = gym.make('FrozenLake-v0')

    def get_array(self,s):
        return numpy.identity(16)[s:s + 1]

    def start_env(self):


        for i in range(1,10000):
            s = self.env.reset()
            d = 0
            while not d:
                s1,d = self.make_cycle(s,i)
                s = s1
            if i%200 == 1:
                print(self.rewards,i,(self.rewards/i)*100,self.rewards_last/200)
                self.rewards_last = 0
                self.cycles_last = 0
                weight=self.rl_component.model.get_value_of_weight()

                self.weights.append(weight)
        for i in range(16):
            input = self.get_array(i)
            activations = self.rl_component.model.feed_forward(input)

            best_action = numpy.argmax(activations)
            print(i,best_action)
        #self.print_weights(14)

    def print_weights(self,state):
        #print(self.weights[0][0].reshape([1,4]))
        column_list = ["Left","Down","Right","Up"]
        df = pandas.DataFrame([],columns=column_list)
        for w in self.weights:
            weight = w[state].reshape([1,4])
            df1 = pandas.DataFrame(weight,columns=column_list)
            #print(df1)
            df = df.append(df1,ignore_index=True)
        #print(df)

        ax = df[column_list].plot()
        #plt.show()

        plt.show()
    def make_cycle(self,s,i):

        input = self.get_array(s)
        self.rl_component.check_if_model_is_valid(16, 4)

        activations = self.rl_component.model.feed_forward(input)

        best_action = numpy.argmax(activations)

        self.epsilon = 1. / ((i / 50) + 10)
        if numpy.random.rand(1)<self.epsilon:
            best_action= self.env.action_space.sample()

        #print(input,best_action)
        s1, r, d, _ = self.env.step(best_action)


        #print(s1,r,d)
        self.rewards+=r
        self.cycles+=1

        self.rewards_last += r
        self.cycles_last += 1
        #print(s,s1,best_action)
        #if (s == s1):
            #print"same state" , s,s1,best_action
        #    r = -0.5
        if (d == True) and not r == 1:
            r = -1
        if r == 0:
            r = -0.01
        output=self.get_array(s1)

        self.rl_component.reward_list.append((input,output,best_action,r))

        self.rl_component.update_model()
        return s1, d

if __name__ == '__main__':
    #rostest.rosrun(PKG, 'update_handler_test_node', UpdateHandlerTestSuite)
    fl_test = FrozenLakeTestSuite()
    fl_test.start_env()