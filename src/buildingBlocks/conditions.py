'''
Created on 13.04.2015

@author: stephan
'''

import activators
import warnings

class Condition(object):
    '''
    This class wraps a sensor and an activator to build a precondition
    '''
    _instanceCounter = 0 # static _instanceCounter to get distinguishable names

    def __init__(self, sensor, activator, name = None):
        '''
        Constructor
        '''
        self._name = name if name else "Condition {0}".format(Condition._instanceCounter)
        self._sensor = sensor
        self._activator = activator
        Condition._instanceCounter += 1
        
    @property
    def activator(self):
        return self._activator
    
    @activator.setter
    def activator(self, newActivator):
        if issubclass(newActivator, activators.Activator):
            self._activator = newActivator
        else:
            warnings.warn("That's no activator!")

    
    @property
    def activation(self):
        return self._activator.computeActivation(self._sensor.value)
    
    def __str__(self):
        return "{0} wrapping {1} with {2}".format(self._name, self._sensor, self._activator)
    
    def __repr__(self):
        return str(self)

        