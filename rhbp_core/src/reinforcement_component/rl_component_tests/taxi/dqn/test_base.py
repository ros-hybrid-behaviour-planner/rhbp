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


class BaseTestSuite(object):
    def __init__(self,algorithm = 0, *args, **kwargs):
        #super(UpdateHandlerTestSuite, self).__init__(*args, **kwargs)
        self.rl_address = "test_agent"
        self.resulting_state = numpy.array([[]])

        # algorithm(=0) is per default dqn
        self.rl_component = RLComponent(self.rl_address,algorithm)

        self.set_up_environment()

    def init_variables(self,seed):

        # set random seeds
        self.env.seed(seed)
        numpy.random.seed(seed)
        # parameter for episodes
        self.max_episodes = 13500
        # metrics for saving results
        self.rewards = 0
        self.cycles = 0
        self.rewards_last = 0
        self.cycles_last = 0
        self.last_r = 0
        self.last_action = 0
        self.counter = 0
        self.rewards_all = 0
        self.counter_last = 0
        self.rewards_tuples=None
        #dqn parameters for random execution
        self.pre_train = 32
        self.startE = 1
        self.endE = 0.0
        self.anneling_steps = 200000
        self.epsilon = self.startE
        self.stepDrop = (self.startE - self.endE) / self.anneling_steps

    def set_up_environment(self):

        self.env = gym.make('Taxi-v2')

        self.num_inputs = 500
        self.num_outputs = 6

    def get_array(self,s):
        return numpy.identity(self.num_inputs)[s:s + 1]

    def start_env(self,num_prints=100,threshold=10,random_seed=0,should_print=True):
        self.init_variables(random_seed)

        for i in range(1,self.max_episodes):
            s = self.env.reset()
            d = 0
            while not d:
                s1,d = self.make_cycle(s,i)
                s = s1
            self.cycles_last+=1
            if self.cycles_last == num_prints:

                # save metrics for this cylce  # TODO make more elegant?
                arr=numpy.array([[self.rewards_last / float(num_prints), self.counter, i]])
                if self.rewards_tuples is None:
                    self.rewards_tuples = arr
                else:
                    self.rewards_tuples = numpy.concatenate( (self.rewards_tuples,arr),axis=0 )

                # print metrics for last cycle if should_print is True
                if should_print:
                    print(self.counter,self.rewards_all/float(self.counter_last),self.rewards,i,(self.rewards/i)*100,self.rewards_last/float(num_prints))
                # if last reward is over threshold print time after stop process
                if self.rewards_last/float(num_prints) > threshold:
                    print(self.__class__.__name__,"Over threshold",threshold,"after",self.counter,"steps and",i,"episodes")
                    return self.rewards_tuples

                # reset variables for last cycle
                self.rewards_last = 0
                self.cycles_last = 0
                self.counter_last=0
                self.rewards_all=0


    def make_cycle(self,s,i):

        # transform input
        input = self.get_array(s)

        # execute the step for the specific logic
        s1,r,d=self.do_step(input)

        # update metrics
        self.last_r = r
        self.rewards_all += r
        self.rewards+=r
        self.cycles+=1
        self.counter_last+=1
        self.rewards_last += r

        return s1, d

    def do_step(self,input):
       """
       override this function with the logic for exeecuting a step. 
       can differ on exploration, action selection, sending invalid actions, or the algorithm
       :param input: 
       :return: 
       """
       raise NotImplementedError

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

    def is_action_valid(self, s, a):
        locs = [(0, 0), (0, 4), (4, 0), (4, 3)]
        row, col, passenger, dest = self.decode(s)
        if a == 4:
            # if passenger ==4:
            #    return False
            for i in range(4):
                if locs[i][0] == row and locs[i][1] == col:
                    return True
            return False
        if a == 5:
            if not passenger == 4:
                return False
            for i in range(4):
                if locs[i][0] == row and locs[i][1] == col:
                    return True
            return False
        return True

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
