'''
Created on 13.04.2015

@author: stephan
'''

class Sensor(object):
    '''
    This class represents information necessary to make decisions.
    Although it is not abstract it will be barely useful and should be used as base class for actual clever implementations that for example subscribe to ROS topics.
    
    '''
    _instanceCounter = 0

    def __init__(self, name = None):
        '''
        Constructor
        '''
        self._name = name if name else "Sensor {0}".format(Sensor._instanceCounter)
        self._value = None # this is what it's all about. Of course, the type and how it is acquired will change depending on the specific sensor
        Sensor._instanceCounter += 1

    @property
    def value(self):
        return self._value
    
    def update(self, newValue):
        '''
        This method is to refresh the _value. It may serve as a callback for ROS subscriptions or be called from some kind of main loop.
        '''
        self._value = newValue
        
    def __str__(self):
        return self._name
    
    def __repr__(self):
        return str(self)