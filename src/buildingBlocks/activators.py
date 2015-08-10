## The Activators module
#Created on 22.04.2015
#@author: stephan

from __future__ import division # force floating point division when using plain /

class Activator(object):
    '''
    This is the abstract base class for an activator which reacts to a value according to the activation scheme implemented in it
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
    
    def getDirection(self):
        '''
        This method should return the direction that increases activation. +1 means rising, -1 means falling
        '''
        raise NotImplementedError()
    
    def getWish(self, value):
        '''
        This method should return an indicator (float in range [-1, 1]) how much and in what direction the value should change in order to reach more activation.
        1 means the value should become true or increase, 0 it should stay the same and -1 it should decrease or become false.
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
    
    def getDirection(self):
        return 1 if self._desired == True else -1
    
    def getWish(self, value):
        assert isinstance(value, bool)
        if value == self._desired:
            return 0.0
        if self._desired == True:
            return 1.0
        return -1.0
    
    def __str__(self):
        return "Boolean Activator [{0} - {1}] ({2})".format(self._minActivation, self._maxActivation, self._desired)
    
    def __repr__(self):
        return "Boolean Activator"
    

class ThresholdActivator(Activator):
    '''
    This class is an activator that compares a sensor's value (expected to be int or float) to a threshold.
    '''
    
    def __init__(self, thresholdValue, isMinimum = True, valueRange = None, minActivation = 0, maxActivation = 1):
        '''
        Constructor
        '''
        super(ThresholdActivator, self).__init__(minActivation, maxActivation)
        self._threshold = float(thresholdValue)   # This is the threshold Value
        self._isMinimum = isMinimum               # The direction in which the threshold must be crossed in order to be activated
        self._valueRange = valueRange             # This is used to assess value deviation. When the range is known it can be estimated how much a given value differs from a satisfactory one (assumed linear relation)
        
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
    
    @property
    def valueRange(self):
        return self._valueRange
    
    @valueRange.setter
    def valueRange(self, valueRange):
        assert isinstance(valueRange, int) or isinstance(valueRange, float)
        self._valueRange = valueRange
    
    def computeActivation(self, value):
        assert isinstance(value, int) or isinstance(value, float)
        if self._isMinimum:
            return self._maxActivation if value >= self._threshold else self._minActivation
        else:
            return self._maxActivation if value < self._threshold else self._minActivation
    
    def getDirection(self):
        return 1 if self._isMinimum else -1

    def getWish(self, value):
        assert isinstance(value, int) or isinstance(value, float)
        if self.computeActivation(value) == self._maxActivation: # no change is required
            return 0.0
        if self._valueRange:
            return sorted((-1.0, (self._threshold - value) / self._valueRange, 1.0))[1] # return how much is missing clamped to [-1, 1]
        else:
            return float(self.getDirection())
    
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
        assert self.valueRange != 0
        rawActivation = (value - self._zeroActivationValue) / self.valueRange
        return sorted((self._minActivation, rawActivation * self.activationRange + self._minActivation, self._maxActivation))[1] # clamp to activation range
    
    def getDirection(self):
        return 1 if self._fullActivationValue > self._zeroActivationValue else -1
    
    def getWish(self, value):
        assert isinstance(value, int) or isinstance(value, float)
        assert self.valueRange != 0
        if self.getDirection() > 0:
            return sorted((0.0, (self._fullActivationValue - value) / abs(self.valueRange), 1.0))[1] # return how much is missing clamped to [0, 1]
        else:
            return sorted((-1.0, (self._fullActivationValue - value) / abs(self.valueRange), 0.0))[1] # return how much is there more than desired clamped to [-1, 0]
            
    @property
    def valueRange(self):
        return self._fullActivationValue - self._zeroActivationValue
    
    @property
    def activationRange(self):
        return self._maxActivation - self._minActivation
    
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