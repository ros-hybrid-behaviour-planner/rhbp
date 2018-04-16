from __future__ import division  # force floating point division when using plain /

import numpy
import rospy
import math
import threading
from abc import ABCMeta, abstractmethod

from behaviour_components.behaviours import BehaviourBase
from behaviour_components.sensors import PassThroughTopicSensor, Sensor
from utils.ros_helpers import get_topic_type

from diagnostic_msgs.msg import KeyValue
from rcs_ros_bridge.msg import *




# patrick





class MakeActionBehavior(BehaviourBase):
    """
    A simple behaviour for triggering generic MAPC actions that just need a type and static parameters
    """

    def __init__(self,state_sensor, reward_sensor,action_index, environment, name, params=[], **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param action_type: type of the MAC action
        :param params: optional parameters for the MAC action
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(MakeActionBehavior, self).__init__(name=name, requires_execution_steps=True, **kwargs)
        self.reward_sensor = reward_sensor
        self.sensor = state_sensor
        self.environment = environment
        self.index = action_index
        self._params = params
        self.last_state = 0
        print("init behavior",self.index)
        self.isEnded = False

    def do_step(self):



        try:
            last_state = self.sensor.value
            state,reward,self.isEnded, _ = self.environment.step(self.index)

            if self.isEnded and reward ==0:
                reward=-1

            self.reward_sensor.update(reward)
            #self.reward_sensor.updates_evaluation(reward, self.isEnded)
            self.reward_sensor.updates_evaluation(reward,self.isEnded,reward==-1)

            #print("ex: ", self.index, "last state",last_state,
            #      "new state",state,"with reward:",reward)

            if self.isEnded:
                state = self.environment.reset()
                self.sensor.update(state)

            self.sensor.update(state)
        except Exception:
            return

    def unregister(self, terminate_services=True):
        super(MakeActionBehavior, self).unregister(terminate_services=terminate_services)
        self._pub_generic_action.unregister()




class MakeActionBehaviorTaxi(BehaviourBase):
    """
    A simple behaviour for triggering generic MAPC actions that just need a type and static parameters
    """

    def __init__(self,state_sensor, reward_sensor,action_index, environment, name, params=[], **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param action_type: type of the MAC action
        :param params: optional parameters for the MAC action
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(MakeActionBehaviorTaxi, self).__init__(name=name, requires_execution_steps=True, **kwargs)
        self.reward_sensor = reward_sensor
        self.sensor_row = state_sensor[0]
        self.sensor_col = state_sensor[1]
        self.sensor_passenger = state_sensor[2]
        self.sensor_destination = state_sensor[3]
        self.state_sensor = state_sensor[4]
        self.environment = environment
        self.index = action_index
        self._params = params
        self.last_state = 0
        print("init behavior",self.index)
        self.isEnded = False

    def encode(self, taxirow, taxicol, passloc, destidx):
        # (5) 5, 5, 4
        i = taxirow
        i *= 5
        i += taxicol
        i *= 5
        i += passloc
        i *= 4
        i += destidx
        return i

    def decode(self, i):
        out = []
        out.append(i % 4)  # row
        i = i // 4
        out.append(i % 5)  # col
        i = i // 5
        out.append(i % 5)  # passloc
        i = i // 5
        out.append(i)       # destination
        assert 0 <= i < 5
        return reversed(out)

    def do_step(self):

        try:
            #last_state = self.sensor.value
            state,reward,self.isEnded, _ = self.environment.step(self.index)

            row,col,passenger,destination = self.decode(state)
            #print(state,"=",row,col,passenger,destination)

            self.sensor_row.update(row)
            self.sensor_col.update(col)
            self.sensor_passenger.update(passenger)
            self.sensor_destination.update(destination)
            self.state_sensor.update(state)
            #if self.isEnded and reward ==0:
            #    reward=-1

            self.reward_sensor.update(reward)
            self.reward_sensor.updates_evaluation(reward,self.isEnded)
            #print("ex: ", self.index, "last state",last_state,
            #      "new state",state,"with reward:",reward)

            if self.isEnded:
                state = self.environment.reset()
                row, col, passenger, destination = self.decode(state)
                self.sensor_row.update(row)
                self.sensor_col.update(col)
                self.sensor_passenger.update(passenger)
                self.sensor_destination.update(destination)
                self.state_sensor.update(state)
            #self.sensor.update(state)
        except Exception as e:
            print(e)
            return

    def unregister(self, terminate_services=True):
        super(MakeActionBehaviorTaxi, self).unregister(terminate_services=terminate_services)
        self._pub_generic_action.unregister()



