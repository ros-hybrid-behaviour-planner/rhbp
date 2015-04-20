'''
Created on 13.04.2015

@author: stephan
'''
from __future__ import division # force floating point division when using plain /

class activator(object):
    '''
    This is an abstract base class for an activator.
    Its actual implementations are supposed to evaluates the contribution of the supervised component(s) on a behaviour's activation
    '''
    _minActivation = 0.0
    _maxActivation = 1.0

    def __init__(self):
        '''
        Constructor
        '''
    
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
    
    
class sensorValueActivator(activator):
    '''
    This activator wraps around a sensor and its derived classes are supposed to work with this sensor
    '''
    _sensor = None
    
    def __init__(self, sensor):
        '''
        Constructor
        '''
        self._sensor = sensor

class thresholdActivator(sensorValueActivator):
    '''
    This class is an activator that compares a sensor's value (expected to be int or float) to a threshold and evaluates the activation level using a margin.
    '''
    _thresholdValue = 0.0 # Activation raises linearly between this value and _fullActivationValue (it remains 0 until here))
    _fullActivationValue = 1.0 # This value (and other values further away from _threshold in this direction) means total activation
    
    def __init__(self, sensor, thresholdValue, fullActivationValue):
        super(thresholdActivator, self).__init__(sensor)
        self._thresholdValue = thresholdValue
        self._fullActivationValue = fullActivationValue
    
    
    def getActivation(self):
        value = self._sensor.getValue()
        assert isinstance(value, int) or isinstance(value, float)
        valueRange = self._fullActivationValue - self._thresholdValue
        assert valueRange != 0
        rawActivation = (value - self._thresholdValue) / valueRange
        activationRange = self._maxActivation - self._minActivation
        assert self._minActivation <= self._maxActivation
        return max(min(self._maxActivation, rawActivation * activationRange + self._minActivation), self._minActivation) # clamp to activation range
        
class goalActivator(activator):
    '''
    This activator wraps around a goal spreads activation from the goal to correlating behaviours.
    Possibly, this is going to be merged with sensorActivator as it is actually the same but backwards.
    '''   
    _goal = None 
    
    def __init__(self, goal):
        '''
        Constructor
        '''
        self._goal = goal