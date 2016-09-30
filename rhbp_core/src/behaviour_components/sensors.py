'''
Created on 13.04.2015

@author: wypler, hrabia
'''

import rospy
import pddl
from utils.ros_helpers import get_topic_type

class Sensor(object):
    '''
    This class represents information necessary to make decisions.
    Although it is not abstract it will be barely useful and should be used as base class for actual clever implementations that for planner_node subscribe to ROS topics.
    
    '''
    _instanceCounter = 0

    def __init__(self, name = None, optional = False, initial_value = None):
        '''
        Constructor
        '''
        self._name = name if name else "Sensor_{0}".format(Sensor._instanceCounter)
        self._name = pddl.create_valid_pddl_name(self._name)
        self._optional = optional
        self._value = initial_value # this is what it's all about. Of course, the type and how it is acquired will change depending on the specific sensor
        self._latestValue = initial_value

        Sensor._instanceCounter += 1

    def sync(self):
        '''
        Keep a explicit copy of the current value
        returns the just stored value
        '''
        self._value = self._latestValue
        return  self._value

    def update(self, newValue):
        '''
        This method is to refresh the _value.
        '''
        self._latestValue = newValue

    @property
    def value(self):
        return self._value

    @property
    def latestValue(self):
        return self._latestValue

    @property
    def optional(self):
        return self._optional
    
    @optional.setter
    def optional(self, newValue):
        if not isinstance(newValue, bool):
            rospy.logwarn("Passed non-Bool value to 'optional' attribute of sensor %s. Parameter was %s", self._name, newValue)
        else:
            self._optional = newValue
        
    def __str__(self):
        return self._name
    
    def __repr__(self):
        return str(self)

    @property
    def name(self):
        return self._name
    
    @name.setter
    def name(self, newName):
        self._name = newName

class SimpleTopicSensor(Sensor):
    """
    "simple" because apparently only primitive message types like Bool and Float have their actual value in a "data" attribute.
    :param topic: topic name to subscribe to
    :param name: name of the sensor, if None a name is generated from the topic
    :param message_type: if not determined an automatic type determination is attempted, requires the topic already be registered on the master
    """
    def __init__(self, topic, name=None, message_type = None, initial_value = None, create_log = False):

        if name is None:
            if topic is None:
                raise ValueError("Invalid name and topic")
            else :
                name = pddl.create_valid_pddl_name(topic)

        super(SimpleTopicSensor, self).__init__(name=name, initial_value=initial_value)

        # if the type is not specified, try to detect it automatically
        if message_type is None:
            messageType = get_topic_type(topic)

        if messageType is not None:
            self._sub = rospy.Subscriber(topic, messageType, self.subscription_callback)
            self._iShouldCreateLog = create_log
            if self._iShouldCreateLog:
                self._logFile = open("{0}.log".format(self._name), 'w')
                self._logFile.write('{0}\n'.format(self._name))
        else:
            rospy.logerr("Could not determine message type of: " + topic)
    
    def subscription_callback(self, msg):
        self.update(msg.data)
        rospy.logdebug("%s received sensor message: %s of type %s", self._name, self.value, type(self.value))
        if self._iShouldCreateLog:
            self._logFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._value))
            self._logFile.flush()

class PassThroughTopicSensor(SimpleTopicSensor):
    """
    "PassThrough" because the sensor just forwards the received msg
    """
    def __init__(self, name, topic, message_type = None,  initial_value = None, create_log = False):
        super(PassThroughTopicSensor, self).__init__(topic=topic, name = name, message_type = message_type, initial_value = initial_value, create_log=create_log)

    def subscription_callback(self, msg):
        self.update(msg)
        rospy.logdebug("%s received sensor message: %s of type %s", self._name, self.value, type(self.value))
        if self._iShouldCreateLog:
            self._logFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._value))
            self._logFile.flush()
