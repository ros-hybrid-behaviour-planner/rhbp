'''
Created on 13.04.2015

@author: stephan
'''
from __future__ import division # force floating point division when using plain /

class Activator(object):
    '''
    This is an abstract base class for an activator.
    Its actual implementations are supposed to evaluate the contribution of the supervised component(s) on a behaviour's activation.
    '''
    _instanceCounter = 0 # static _instanceCounter to get distinguishable names

    def __init__(self, name = None):
        '''
        Constructor
        '''
        self._name = name if name else "Activator {0}".format(Activator._instanceCounter)
        self._minActivation = 0.0
        self._maxActivation = 1.0
        Activator._instanceCounter += 1
    
    def setMinActivation(self, minActivationLevel = 0.0):
        self._minActivation = float(minActivationLevel)
        
    def setMaxActivation(self, maxActivationLevel = 1.0):
        self._maxActivation = float(maxActivationLevel)
    
    def getActivation(self):
        '''
        This method should return an activation level between _minActivation (no activation) and _maxActivation (total activation)
        The actual implementation obviously depends on the sensor so that there is no useful default here.
        '''
        raise NotImplementedError()
    
    def __str__(self):
        return self._name
    
    
class SensorValueActivator(Activator):
    '''
    This activator wraps around a sensor and its derived classes are supposed to work with this sensor
    '''
    _sensor = None
    
    def __init__(self, sensor, name = None):
        '''
        Constructor
        '''
        super(SensorValueActivator, self).__init__(name)
        self._name = name if name else "SensorValueActivator {0}".format(Activator._instanceCounter)
        self._sensor = sensor

class ThresholdActivator(SensorValueActivator):
    '''
    This class is an activator that compares a sensor's value (expected to be int or float) to a threshold and evaluates the activation level using a margin.
    '''
    
    def __init__(self, sensor, thresholdValue, fullActivationValue, name = None):
        super(ThresholdActivator, self).__init__(sensor, name)
        self._name = name if name else "ThresholdActivator {0}".format(Activator._instanceCounter)
        self._thresholdValue = thresholdValue           # Activation raises linearly between this value and _fullActivationValue (it remains 0 until here))
        self._fullActivationValue = fullActivationValue # This value (and other values further away from _threshold in this direction) means total activation
    
    
    def getActivation(self):
        value = self._sensor.getValue()
        assert isinstance(value, int) or isinstance(value, float)
        valueRange = self._fullActivationValue - self._thresholdValue
        assert valueRange != 0
        rawActivation = (value - self._thresholdValue) / valueRange
        activationRange = self._maxActivation - self._minActivation
        assert self._minActivation <= self._maxActivation
        return max(min(self._maxActivation, rawActivation * activationRange + self._minActivation), self._minActivation) # clamp to activation range
        
class GoalActivator(Activator):
    '''
    This activator wraps around a goal spreads activation from the goal to correlating behaviours.
    Possibly, this is going to be merged with sensorActivator as it is actually the same but backwards.
    '''   
    _goal = None 
    
    def __init__(self, goal, name = None):
        '''
        Constructor
        '''
        super(Activator, self).__init__(name)
        self._name = name if name else "GoalActivator {0}".format(Activator._instanceCounter)
        self._goal = goal