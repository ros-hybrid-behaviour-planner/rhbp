'''
Created on 13.04.2015

@author: wypler, hrabia
'''

import rospy

class Sensor(object):
    '''
    This class represents information necessary to make decisions.
    Although it is not abstract it will be barely useful and should be used as base class for actual clever implementations that for planner_node subscribe to ROS topics.
    
    '''
    _instanceCounter = 0

    def __init__(self, name = None, optional = False, initialValue = None):
        '''
        Constructor
        '''
        self._name = name if name else "Sensor {0}".format(Sensor._instanceCounter)
        self._optional = optional
        self._value = initialValue # this is what it's all about. Of course, the type and how it is acquired will change depending on the specific sensor
        self._syncValue = initialValue

        Sensor._instanceCounter += 1

    def sync(self):
        '''
        Keep a explicit copy of the current value
        returns the just stored value
        '''
        self._syncValue = self._value
        return  self._syncValue

    def update(self, newValue):
        '''
        This method is to refresh the _value.
        '''
        self._value = newValue

    @property
    def value(self):
        return self._value

    @property
    def valueSync(self):
        return self._syncValue

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
    """
    def __init__(self, name, topic, messageType, initialValue = None, createLog = False):
        super(SimpleTopicSensor, self).__init__(name = name, initialValue = initialValue)
        self._sub = rospy.Subscriber(topic, messageType, self.subscriptionCallback)
        self._iShouldCreateLog = createLog
        if self._iShouldCreateLog:
            self._logFile = open("{0}.log".format(self._name), 'w')
            self._logFile.write('{0}\n'.format(self._name))
    
    def subscriptionCallback(self, msg):
        self.update(msg.data)
        rospy.logdebug("%s received sensor message: %s of type %s", self._name, self.value, type(self.value))
        if self._iShouldCreateLog:
            self._logFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._value))
            self._logFile.flush()

class PassThroughTopicSensor(Sensor):
    """
    "PassThrough" because the sensor just forwards the received msg
    """
    def __init__(self, name, topic, messageType, initialValue = None, createLog = False):
        super(PassThroughTopicSensor, self).__init__(name = name, initialValue = initialValue)
        self._sub = rospy.Subscriber(topic, messageType, self.subscriptionCallback)
        self._iShouldCreateLog = createLog
        if self._iShouldCreateLog:
            self._logFile = open("{0}.log".format(self._name), 'w')
            self._logFile.write('{0}\n'.format(self._name))
    
    def subscriptionCallback(self, msg):
        self.update(msg)
        rospy.logdebug("%s received sensor message: %s of type %s", self._name, self.value, type(self.value))
        if self._iShouldCreateLog:
            self._logFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._value))
            self._logFile.flush()
