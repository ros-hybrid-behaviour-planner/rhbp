'''
Created on 22.04.2015

@author: stephan
'''

from __future__ import division # force floating point division when using plain /



class Activator(object):
    '''
    This is the abstract base class for an activator which reacts to a value according to the activation behaviour implemented in it
    '''

    def __init__(self, minActivation = 0, maxActivation = 1):
        '''
        Constructor
        '''
        self._minActivation = float(minActivation)
        self._maxActivation = float(maxActivation)
        
    @property
    def minActivation(self):
        return self._minActivation

    @minActivation.setter
    def minActivation(self, minActivationLevel):
        self._minActivation = float(minActivationLevel)
    
    @property
    def maxActivation(self):
        return self._maxActivation

    @maxActivation.setter
    def maxActivation(self, maxActivationLevel):
        self._maxActivation = float(maxActivationLevel)    
    
    def computeActivation(self, value):
        '''
        This method should return an activation level.
        The actual implementation obviously depends on the type of activation so that there is no useful default here.
        '''
        raise NotImplementedError()
    
    def __str__(self):
        return "Activator"
    
    def __repr__(self):
        return str(self)
    

class BooleanActivator(Activator):
    '''
    This class is an activator that compares a boolean value to a desired value
    '''
    
    def __init__(self, desiredValue = True, minActivation = 0, maxActivation = 1):
        '''
        Constructor
        '''
        super(BooleanActivator, self).__init__(minActivation, maxActivation)
        self._desired = desiredValue   # this is the threshold Value
    
    
    
    def computeActivation(self, value):
        assert isinstance(value, bool)
        return self._maxActivation if value == self._desired else self._minActivation
    
    def __str__(self):
        return "Boolean Activator [{0} - {1}] ({2})".format(self._minActivation, self._maxActivation, self._desired)
    
    def __repr__(self):
        return "Boolean Activator"
    

class ThresholdActivator(Activator):
    '''
    This class is an activator that compares a sensor's value (expected to be int or float) to a threshold.
    '''
    
    def __init__(self, thresholdValue, isMinimum = True, minActivation = 0, maxActivation = 1):
        '''
        Constructor
        '''
        super(ThresholdActivator, self).__init__(minActivation, maxActivation)
        self._threshold = float(thresholdValue)   # this is the threshold Value
        self._isMinimum = isMinimum               # The direction in which the threshold must be crossed in order to be activated
        
    @property
    def threshold(self):
        return self._threshold
    
    @threshold.setter
    def threshold(self, thresholdValue):
        assert isinstance(thresholdValue, int) or isinstance(thresholdValue, float)
        self._fullActivationValue = float(thresholdValue);
    
    @property
    def minimum(self):
        return self._isMinimum
    
    @minimum.setter
    def minimum(self, mini):
        assert isinstance(mini, bool)
        self._isMinimum = mini
    
    def computeActivation(self, value):
        assert isinstance(value, int) or isinstance(value, float)
        if self._isMinimum:
            return self._maxActivation if value >= self._threshold else self._minActivation
        else:
            return self._maxActivation if value < self._threshold else self._minActivation
        
    def __str__(self):
        return "Threshold Activator [{0} - {1}] ({2} or {3})".format(self._minActivation, self._maxActivation, self._threshold, "above" if self._isMinimum else "below")
    
    def __repr__(self):
        return "Threshold Activator"

class LinearActivator(Activator):
    '''
    This class is an activator that takes a value (expected to be int or float) and computes a linear slope of activation within a value range.
    '''

    def __init__(self, zeroActivationValue, fullActivationValue, minActivation = 0, maxActivation = 1):
        '''
        Constructor
        '''
        super(LinearActivator, self).__init__(minActivation, maxActivation)
        self._zeroActivationValue = float(zeroActivationValue) # Activation raises linearly between this value and _fullActivationValue (it remains 0 until here))
        self._fullActivationValue = float(fullActivationValue) # This value (and other values further away from _threshold in this direction) means total activation
    
    def computeActivation(self, value):
        assert isinstance(value, int) or isinstance(value, float)
        valueRange = self._fullActivationValue - self._zeroActivationValue
        assert valueRange != 0
        rawActivation = (value - self._zeroActivationValue) / valueRange
        activationRange = self._maxActivation - self._minActivation
        assert self._minActivation <= self._maxActivation
        return max(min(self._maxActivation, rawActivation * activationRange + self._minActivation), self._minActivation) # clamp to activation range
    
    @property
    def zeroActivationValue(self):
        return self._zeroActivationValue
    
    @zeroActivationValue.setter
    def zeroActivationValue(self, value):
        assert isinstance(value, int) or isinstance(value, float)
        self._zeroActivationValue = float(value);
    
    @property
    def fullActivationValue(self):
        return self._fullActivationValue
    
    @fullActivationValue.setter
    def fullActivationValue(self, value):
        assert isinstance(value, int) or isinstance(value, float)
        self._fullActivationValue = float(value);
    
    def __str__(self):
        return "Linear Activator [{0} - {1}] ({2} - {3})".format(self._minActivation, self._maxActivation, self._zeroActivationValue, self._fullActivationValue)
    
    def __repr__(self):
        return "Linear Activator"