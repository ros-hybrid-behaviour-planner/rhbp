from __future__ import division  # force floating point division when using plain /
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

    def __init__(self,state_sensor, action_index, environment, name, params=[], **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param action_type: type of the MAC action
        :param params: optional parameters for the MAC action
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(MakeActionBehavior, self).__init__(name=name, requires_execution_steps=True, **kwargs)

        self.sensor = state_sensor
        self.environment = environment
        self.index = action_index
        self._params = params


    def do_step(self):

        state,reward,isEnded, _ = self.environment.step(self.index)
        print("execute behavior index: ", self.index, "with reward:",reward)
        if isEnded:
            state = self.environment.reset()
        self.sensor.update(state)


    def unregister(self, terminate_services=True):
        super(MakeActionBehavior, self).unregister(terminate_services=terminate_services)
        self._pub_generic_action.unregister()



