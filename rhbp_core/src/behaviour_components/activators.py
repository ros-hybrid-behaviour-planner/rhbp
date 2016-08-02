## The Activators module
#Created on 22.04.2015
#@author: wypler, hrabia

from __future__ import division # force floating point division when using plain /
from util import PDDL
import rospy

from conditions import Conditonal
  
class Condition(Conditonal):
    '''
    This is the basic Condition class it brings together the sensor and the activation function and takes care
    of the processing and caching of calculated data
    '''    
    _instanceCounter = 0 # static _instanceCounter to get distinguishable names

    def __init__(self, sensor, activator, name = None):
        '''
        Constructor
        '''
        super(Conditonal, self).__init__()
        self._name = name if name else "Condition {0}".format(Condition._instanceCounter)
        self._sensor = sensor
        self._activator = activator

        self._normalizedSensorValue = 0
        self._satisfaction = 0

    def sync(self):
        self._sensor.sync()

    def updateComputation(self):
        '''
        This method needs to be executed at first on every decision cycle before accessing all other values/results/methods
        '''
        self._normalizedSensorValue = self._normalize()

        try:
            self._satisfaction = self._activator.computeActivation(self._normalizedSensorValue)
        except AssertionError:
            rospy.logwarn("Wrong data type for %s in %s. Got %s. Possibly uninitialized%s sensor %s?", self._sensor,
                          self._name, type(self._sensor.value), " optional" if self._sensor.optional else "",
                          self._sensor.name)
            raise

    def _normalize(self):
        '''
        Normalize the current sensor value into a floating point number
        '''
        if self._sensor:
            return self._sensor.value()
        else:
           return 0

    def getWishes(self):
        '''
        returns a list of wishes (a wish is a tuple (sensor, indicator <float> [-1, 1]).
        Well, there is only one wish from one sensor - activator pair here but to keep a uniform interface with conjunction and disjunction this method wraps them into a list.
        '''
        try:
            return [(self._sensor, self._activator.getSensorWish(self._normalizedSensorValue))]
        except AssertionError:
            rospy.logwarn("Wrong data type for %s in %s. Got %s. Possibly uninitialized%s sensor %s?", self._sensors, self._name, type(self._sensors.value), " optional" if self._sensors.optional else "", self._sensors.name)
            raise
    
    def getPreconditionPDDL(self):
        return self._activator.getSensorPreconditionPDDL(self._sensor.name)
    
    def getStatePDDL(self):
        return [self._activator.getSensorStatePDDL(self._sensor.name, self._normalizedSensorValue)]

    @property
    def satisfaction(self):
        '''
        This property specifies to what extend the condition is fulfilled.
        '''
        return self._satisfaction

    # TODO check if we really need the following property!!!!
    @property
    def sensor(self):
        rospy.logwarn("called sensor optional")
        return self._sensor

    @property
    def optional(self):
        return self._sensor.optional
        
    def __str__(self):
        return "{0} {{{1} : v: {2}, s: {3}}}".format(self._name, self._sensor, self._normalizedSensorValue, self._satisfaction)
    
    def __repr__(self):
        return str(self)

class MultiSensorCondition(Condition):
    '''
    This is a special abstract condition class supporting multiple sensor instances
    The normalization can be implemented sensor type specific by subclassing
    and overwriting the normalize method
    The _reduceSatisfaction method has to be implemented in order combine the different sensors
    '''
    _instanceCounter = 0 # static _instanceCounter to get distinguishable names

    def __init__(self, sensors, activator, name = None):
        '''
        Constructor
        '''
        self._name = name if name else "MultiSensorCondition {0}".format(Condition._instanceCounter)

        assert hasattr(sensors, '__getitem__'), "sensors is not a tuple or list"
        self._sensors = sensors
        self._activator = activator

        self._normalizedSensorValues = dict.fromkeys(self._sensors, 0)
        self._sensorSatisfactions = dict.fromkeys(self._sensors, 0)

        self._satisfaction = 0


    def sync(self):
        for s in self._sensors:
            s.sync()

    def updateComputation(self):
        '''
        This method needs to be executed at first on every decision cycle before accessing all other values/results/methods
        '''
        self._normalize()

        self._computeSatisfactions()

        self._satisfaction = self._reduceSatisfaction()

    def _normalize(self):
        '''
        Normalize the current sensor values resulting in a dict of sensors and normalized values
        This also allows to reduce several sensors into one normalized value
        '''
        for s in self._sensors:
            self._normalizedSensorValues[s] = s.value

    def _computeSatisfactions(self):
        '''
        This method is a base implementation that calculates the satisfaction for each sensor
        It can also be omitted if the _reduceSatisfaction method is not usind the _sensorSatisfactions dict
        '''
        for s in self._sensors:
            self._sensorSatisfactions[s] = self._activator.computeActivation(self._normalizedSensorValues[s])

    def _reduceSatisfaction(self):
        '''
        This class has to be implemented specific to used sensors and application
        It has to return a new satisfaction value
        '''
        raise NotImplementedError()

    def getWishes(self):
        '''
        returns a list of wishes (a wish is a tuple (sensor, indicator <float> [-1, 1]).
        Well, there is only one wish from one sensor - activator pair here but to keep a uniform interface with conjunction and disjunction this method wraps them into a list.
        '''
        try:
            return [(s, self._activator.getSensorWish(self._normalizedSensorValues[s])) for s in self._sensors]
        except AssertionError:
            rospy.logwarn("Wrong data type for %s in %s. Got %s. Possibly uninitialized%s sensor %s?", self._sensors, self._name, type(self._sensors.value), " optional" if self._sensors.optional else "", self._sensors.name)
            raise

    def getPreconditionPDDL(self):
        # Calling getSensorPreconditionPDDL for all sensors
        conditions = [self._activator.getSensorPreconditionPDDL(s.name) for s in self._sensors]

        # TODO this code is copy pasted from conjuction condition
        pddl = PDDL(statement = "(and")
        for cond_pddl in conditions:
            pddl.statement += " {0}".format(cond_pddl.statement)
            pddl.predicates = pddl.predicates.union(cond_pddl.predicates)
            pddl.functions = pddl.functions.union(cond_pddl.functions)
        pddl.statement += ")"
        return pddl


    def getStatePDDL(self):
        #Calling getSensorStatePDDL for all sensors
        return [self._activator.getSensorStatePDDL(s.name, self._normalizedSensorValues[s]) for s in self._sensors]

    @property
    def optional(self):
        return reduce(lambda x, y: x and y, self._sensors, False)

    def __str__(self):
        return "{0} {1}".format(self._name, self.satisfaction)

    def __repr__(self):
        return str(self)


class Activator(object):
    '''
    This is the abstract base class for an activator which reacts to a value according to the activation scheme implemented in it
    '''
    _instanceCounter = 0  # static _instanceCounter to get distinguishable names

    def __init__(self, minActivation=0, maxActivation=1, name=None):
        '''
        Constructor
        '''
        self._name = name if name else "Activator {0}".format(Activator._instanceCounter)
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

    def computeActivation(self, normalizedValue):
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

    def getSensorWish(self, normalizedValue):
        '''
        This method should return an indicator (float in range [-1, 1]) how much and in what direction the value should change in order to reach more activation.
        1 means the value should become true or increase, 0 it should stay the same and -1 it should decrease or become false.
        '''
        raise NotImplementedError()

    def getSensorPreconditionPDDL(self, sensorName):
        '''
        This method should produce valid PDDL condition expressions suitable for FastDownward (http://www.fast-downward.org/PddlSupport)
        '''
        raise NotImplementedError()

    def getSensorStatePDDL(self, sensorName, normalizedValue):
        '''
        This method should produce a valid PDDL statement describing the current state (the (normalized) value) of sensor sensorName in a form suitable for FastDownward (http://www.fast-downward.org/PddlSupport)
        '''
        raise NotImplementedError()


    def __str__(self):
        return "{0}".format(self._name)

    def __repr__(self):
        return str(self)
    

class BooleanActivator(Activator):
    '''
    This class is an activator that compares a boolean value to a desired value
    '''
    
    def __init__(self, desiredValue = True, minActivation = 0, maxActivation = 1, name = None):
        '''
        Constructor
        '''
        super(BooleanActivator, self).__init__(minActivation, maxActivation, name)
        self._desired = desiredValue   # this is the threshold Value
    
    def computeActivation(self, normalizedValue):
        assert isinstance(normalizedValue, bool)
        return self._maxActivation if normalizedValue == self._desired else self._minActivation
    
    def getDirection(self):
        return 1 if self._desired == True else -1

    def getSensorWish(self, normalizedValue):
        assert isinstance(normalizedValue, bool)
        if normalizedValue == self._desired:
            return 0.0
        if self._desired == True:
            return 1.0
        return -1.0
    
    def getSensorPreconditionPDDL(self, sensorName):
        return PDDL(statement = "(" + sensorName + ")" if self._desired == True else "(not (" + sensorName + "))", predicates = sensorName)

    def getSensorStatePDDL(self, sensorName, normalizedValue):
        return PDDL(statement = "(" + sensorName + ")" if normalizedValue == True else "(not (" + sensorName + "))", predicates = sensorName)
        
    def __str__(self):
        return "Boolean Activator [{0} - {1}] ({2})".format(self._minActivation, self._maxActivation, self._desired)
    
    def __repr__(self):
        return "Boolean Activator"
    

class ThresholdActivator(Activator):
    '''
    This class is an activator that compares a sensor's value (expected to be int or float) to a threshold.
    '''
    def __init__(self, thresholdValue, isMinimum = True, valueRange = None, minActivation = 0, maxActivation = 1, name = None):
        '''
        Constructor
        '''
        super(ThresholdActivator, self).__init__(minActivation, maxActivation, name)
        self._threshold = float(thresholdValue)   # This is the threshold Value
        self._isMinimum = isMinimum               # The direction in which the threshold must be crossed in order to be activated
        self._valueRange = valueRange             # This is used to assess value deviation. When the range is known it can be estimated how much a given value differs from a satisfactory one (assumed linear relation)
        
    @property
    def threshold(self):
        return self._threshold
    
    @threshold.setter
    def threshold(self, thresholdValue):
        assert isinstance(thresholdValue, int) or isinstance(thresholdValue, float)
        self._threshold = float(thresholdValue);
    
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
    
    def computeActivation(self, normalizedValue):
        assert isinstance(normalizedValue, int) or isinstance(normalizedValue, float)
        if self._isMinimum:
            return self._maxActivation if normalizedValue >= self._threshold else self._minActivation
        else:
            return self._maxActivation if normalizedValue <= self._threshold else self._minActivation
    
    def getDirection(self):
        return 1 if self._isMinimum else -1

    def getSensorWish(self, normalizedValue):
        assert isinstance(normalizedValue, int) or isinstance(normalizedValue, float)
        if self.computeActivation(normalizedValue) == self._maxActivation: # no change is required
            return 0.0
        if self._valueRange:
            return sorted((-1.0, (self._threshold - normalizedValue) / self._valueRange, 1.0))[1] # return how much is missing clamped to [-1, 1]
        else:
            return float(self.getDirection())

    def getSensorPreconditionPDDL(self, sensorName):
        return PDDL(statement = "( >= (" + sensorName + ") {0} )".format(self._threshold) if self.getDirection() == 1 else "( <= (" + sensorName + ") {0} )".format(self._threshold), functions = sensorName)
    
    def getSensorStatePDDL(self, sensorName, normalizedValue):
        return PDDL(statement = "( = (" + sensorName + ") {0} )".format(normalizedValue), functions = sensorName)
    
    def __str__(self):
        return "Threshold Activator [{0} - {1}] ({2} or {3})".format(self._minActivation, self._maxActivation, self._threshold, "above" if self._isMinimum else "below")
    
    def __repr__(self):
        return "Threshold Activator"

class LinearActivator(Activator):
    '''
    This class is an activator that takes a value (expected to be int or float) and computes a linear slope of activation within a value range.
    '''
    def __init__(self, zeroActivationValue, fullActivationValue, minActivation = 0, maxActivation = 1, name = None):
        '''
        Constructor
        '''
        super(LinearActivator, self).__init__(minActivation, maxActivation, name)
        self._zeroActivationValue = float(zeroActivationValue) # Activation raises linearly between this value and _fullActivationValue (it remains 0 until here))
        self._fullActivationValue = float(fullActivationValue) # This value (and other values further away from _threshold in this direction) means total activation
    
    def computeActivation(self, normalizedValue):
        assert isinstance(normalizedValue, int) or isinstance(normalizedValue, float)
        assert self.valueRange != 0
        rawActivation = (normalizedValue - self._zeroActivationValue) / self.valueRange
        return sorted((self._minActivation, rawActivation * self.activationRange + self._minActivation, self._maxActivation))[1] # clamp to activation range
    
    def getDirection(self):
        return 1 if self._fullActivationValue > self._zeroActivationValue else -1

    def getSensorWish(self, normalizedValue):
        assert isinstance(normalizedValue, int) or isinstance(normalizedValue, float)

        assert self.valueRange != 0
        if self.getDirection() > 0:
            return sorted((0.0, (self._fullActivationValue - normalizedValue) / abs(self.valueRange), 1.0))[1] # return how much is missing clamped to [0, 1]
        else:
            return sorted((-1.0, (self._fullActivationValue - normalizedValue) / abs(self.valueRange), 0.0))[1] # return how much is there more than desired clamped to [-1, 0]
        
    def getSensorPreconditionPDDL(self, sensorName):
        return PDDL(statement = "( >= (" + sensorName + ") {0} )".format(self._zeroActivationValue) if self.getDirection() == 1 else "( <= (" + sensorName + ") {0} )".format(self._zeroActivationValue), functions = sensorName)  # TODO: This is not actually correct: The lower bound is actually not satisfying. How can we do better?

    def getSensorStatePDDL(self, sensorName, normalizedValue):
        return PDDL(statement = "( = (" + sensorName + ") {0} )".format(normalizedValue), functions = sensorName)

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