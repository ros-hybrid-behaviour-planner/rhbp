'''
Created on 13.04.2015

@author: stephan
'''

import rospy

class Sensor(object):
    '''
    This class represents information necessary to make decisions.
    Although it is not abstract it will be barely useful and should be used as base class for actual clever implementations that for example subscribe to ROS topics.
    
    '''
    _instanceCounter = 0

    def __init__(self, name = None, optional = False):
        '''
        Constructor
        '''
        self._name = name if name else "Sensor {0}".format(Sensor._instanceCounter)
        self._optional = optional
        self._value = None # this is what it's all about. Of course, the type and how it is acquired will change depending on the specific sensor
        Sensor._instanceCounter += 1

    @property
    def value(self):
        return self._value
    
    def update(self, newValue):
        '''
        This method is to refresh the _value.
        '''
        self._value = newValue
        
    @property
    def optional(self):
        return self._optional
    
    @optional.setter
    def optional(self, newValue):
        if not isinstance(newValue, Bool):
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

class TopicSensor(Sensor):
    def __init__(self, name, topic, messageType, createLog = False):
        super(TopicSensor, self).__init__(name)
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