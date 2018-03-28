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


class TaxiTestSuite():
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
        self.env = gym.make('Taxi-v2')
        print(self.env.action_space)
        print(self.env.observation_space)
        self.num_inputs = 19
        self.num_outputs = 6

    def get_array(self,s):
        #return numpy.identity(500)[s:s + 1]
        taxirow, taxicol, passidx, destidx = self.decode(s)
        has_passenger = passidx<4
        a1=numpy.identity(5)[taxirow:taxirow + 1]
        a2=numpy.identity(5)[taxicol:taxicol + 1]
        a3=numpy.identity(5)[passidx:passidx + 1]
        a4=numpy.identity(4)[destidx:destidx + 1]

        array = numpy.concatenate([a1,a2,a3,a4],axis=1)
        #print(array)
        #array = numpy.array([[taxirow, taxicol, passidx, destidx , has_passenger]])
        return array


        #    outfile.write("  ({})\n".format(["South", "North", "East", "West", "Pickup", "Dropoff"][self.lastaction]))

    def decode(self, i):
        out = []
        out.append(i % 4)
        i = i // 4
        out.append(i % 5)
        i = i // 5
        out.append(i % 5)
        i = i // 5
        out.append(i)
        assert 0 <= i < 5

        return reversed(out)



    def start_env(self):

        num_prints=100
        for i in range(1,20001):
            s = self.env.reset()
            d = 0
            while not d:
                s1,d = self.make_cycle(s,i)
                s = s1

            if i%num_prints == 1:
                print(self.rewards,i,(self.rewards/i),self.rewards_last/num_prints)
                self.rewards_last = 0
                self.cycles_last = 0
                weight=self.rl_component.model.get_value_of_weight()

                self.weights.append(weight)
        self.print_weights(0)

    def print_weights(self,state):
        column_list=["South", "North", "East", "West", "Pickup", "Dropoff"]
        #print(self.weights[0][0].reshape([1,4]))
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

        self.rl_component.check_if_model_is_valid(self.num_inputs, self.num_outputs)

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


        output=self.get_array(s1)

        self.rl_component.reward_list.append((input,output,best_action,r))

        self.rl_component.update_model()
        return s1, d

if __name__ == '__main__':
    #rostest.rosrun(PKG, 'update_handler_test_node', UpdateHandlerTestSuite)
    fl_test = TaxiTestSuite()
    fl_test.start_env()