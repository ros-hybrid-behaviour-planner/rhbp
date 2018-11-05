## The Activators module
#Created on 22.04.2015
#@author: wypler, hrabia, rieger

from __future__ import division  # force floating point division when using plain /

import utils.rhbp_logging
from behaviour_components.pddl import PDDL, get_pddl_effect_name

rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.conditions')


class Activator(object):
    """
    This is the abstract base class for an activator which reacts to a value according to the activation scheme implemented in it
    """
    _instanceCounter = 0  # static _instanceCounter to get distinguishable names

    def __init__(self, minActivation=0, maxActivation=1, name=None):
        """
        Constructor
        """
        self._name = name if name else "Activator{0}".format(Activator._instanceCounter)
        self._minActivation = float(minActivation)
        self._maxActivation = float(maxActivation)
        Activator._instanceCounter += 1

    def getPDDLFunctionName(self, sensorName):
        return get_pddl_effect_name(sensorName, self._name)

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
        """
        This method should return an activation level.
        The actual implementation obviously depends on the type of activation so that there is no useful default here.
        """
        raise NotImplementedError()

    def getDirection(self):
        """
        Provide information if an increase or decrease of the sensor value is the intended direction for increasing the satisfaction
        :return: +1 for increase and -1 for decrease
        """
        raise NotImplementedError()

    def getSensorWish(self, normalizedValue):
        """
        This method should return an indicator (float in range [-1, 1]) how much and in what direction the value should change in order to reach more activation.
        1 means the value should become true or increase, 0 it should stay the same and -1 it should decrease or become false.
        """
        raise NotImplementedError()

    def getSensorPreconditionPDDL(self, sensorName, satisfaction_threshold, current_sensor_value):
        """
        This method should produce valid PDDL condition expressions suitable for FastDownward (http://www.fast-downward.org/PddlSupport)
        :param sensorName: name of the considered sensor
        :param satisfaction_threshold: threshold value when the conditions becomes valid(true), this hint is not necessarily used, it depends on the activator function, range [0,1]
        :param current_sensor_value: current value of the sensor, this is not necessarily used
        """
        raise NotImplementedError()

    def getSensorStatePDDL(self, sensorName, normalizedValue, sensor_update_time):
        """
        This method should produce a valid PDDL statement describing the current state (the (normalized) value) of sensor sensorName in a form suitable for FastDownward (http://www.fast-downward.org/PddlSupport)
        """
        raise NotImplementedError()

    def __str__(self):
        return "{0}".format(self._name)

    def __repr__(self):
        return str(self)

    @property
    def name(self):
        return self._name


class BooleanActivator(Activator):
    """
    This class is an activator that compares a boolean value to a desired value
    """

    def __init__(self, desiredValue=True, minActivation=0, maxActivation=1, name=None):
        """
        Constructor
        """
        super(BooleanActivator, self).__init__(minActivation, maxActivation, name)
        self._desired = desiredValue   # this is the threshold Value

    def computeActivation(self, normalizedValue):
        assert isinstance(normalizedValue, bool)
        return self._maxActivation if normalizedValue is self._desired else self._minActivation

    def getDirection(self):
        return 1 if self._desired else -1

    def getSensorWish(self, normalizedValue):
        assert isinstance(normalizedValue, bool)
        if normalizedValue == self._desired:
            return 0.0
        if self._desired is True:
            return 1.0
        return -1.0

    def getSensorPreconditionPDDL(self, sensorName, satisfaction_threshold, current_sensor_value):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="(" + functionName + ")" if self._desired is True else "(not (" + functionName + "))", predicates=functionName)

    def getSensorStatePDDL(self, sensorName, normalizedValue, sensor_update_time):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="(" + functionName + ")" if normalizedValue is True else "(not (" + functionName + "))",
                    predicates=functionName, time_stamp=sensor_update_time)

    def __str__(self):
        return "Boolean Activator [{0} - {1}] ({2})".format(self._minActivation, self._maxActivation, self._desired)

    def __repr__(self):
        return "Boolean Activator"


class ThresholdActivator(Activator):
    """
    This class is an activator that compares a sensor's value (expected to be int or float) to a threshold.
    """
    def __init__(self, thresholdValue, isMinimum=True, valueRange=None, minActivation=0, maxActivation=1, name=None):
        """
        :param thresholdValue: This is the threshold Value above/below and equal we get full activation depending on isMinimum
        :param isMinimum: isMinimum=True -> full activation above and equal to threshold, isMinimum=False -> full activation below and equal to threshold
                          isMinimum defines the direction in which the threshold must be passed in order to be activated:
        :param valueRange: This is used to assess value deviation (wish calculation). When the range is known it can be estimated how much a given value differs from a satisfactory one (assumed linear relation)
        :param minActivation: see :class:Activator
        :param maxActivation: see :class:Activator
        :param name: see :class:Activator
        """
        super(ThresholdActivator, self).__init__(minActivation, maxActivation, name)
        self._threshold = float(thresholdValue)
        self._isMinimum = isMinimum
        self._valueRange = valueRange

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

    def getSensorPreconditionPDDL(self, sensorName, satisfaction_threshold, current_sensor_value):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="( >= (" + functionName + ") {0:f} )".format(self._threshold) if self.getDirection() == 1 else "( <= (" + functionName + ") {0:f} )".format(self._threshold), functions = functionName)

    def getSensorStatePDDL(self, sensorName, normalizedValue, sensor_update_time):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="( = (" + functionName + ") {0:f} )".format(normalizedValue), functions=functionName,
                    time_stamp=sensor_update_time)

    def __str__(self):
        return "Threshold Activator [{0} - {1}] ({2} or {3})".format(self._minActivation, self._maxActivation, self._threshold, "above" if self._isMinimum else "below")

    def __repr__(self):
        return "Threshold Activator"


class GreedyActivator(Activator):
    """
    This class is an activator that maximizes or minimizes the current sensor value towards a given offset.
    If the step_size=0 the activator will just be satisfied (not really greedy anymore) with the current sensor value,
    but it will create PDDL conditions that are satisfied with the current value or higher/lower values depending on
    the maximize parameter.
    """

    def __init__(self, maximize=True, step_size=1, name=None):
        """
        :param maximize:
        :param step_size: value, >= 0
        :param name:
        """
        super(GreedyActivator, self).__init__(0, 1, name)
        self.__maximize = maximize
        self.__step_size = step_size

    @property
    def maximize(self):
        return self.__maximize

    def computeActivation(self, normalizedValue):
        if self.__step_size != 0:
            return self.minActivation
        else:
            return self.maxActivation

    def getDirection(self):
        return 1 if self.__maximize else -1

    def getSensorWish(self, normalizedValue):
        return self.getDirection()

    def getSensorPreconditionPDDL(self, sensor_name, satisfaction_threshold, current_value):
        function_name = self.getPDDLFunctionName(sensor_name)
        next_threshold = current_value + self.__step_size * self.getDirection()
        operator = '>=' if self.getDirection() > 0 else '<='
        statement = '( {0} ({1}) {2:f})'.format(operator, function_name, next_threshold)
        return PDDL(statement=statement, functions=function_name)

    def getSensorStatePDDL(self, sensorName, normalizedValue, sensor_update_time):
        function_name = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="( = (" + function_name + ") {0:f} )".format(normalizedValue), functions=function_name,
                    time_stamp=sensor_update_time)

    def __str__(self):
        return "Greedy Activator{0}".format("+" if self.__maximize else "-")

    def __repr__(self):
        return "Greedy Activator"


class LinearActivator(Activator):
    """
    This class is an activator that takes a value (expected to be int or float) and computes a linear slope of activation within a value range.
    """
    def __init__(self, zeroActivationValue, fullActivationValue, minActivation=0, maxActivation=1, name=None):
        """
        Constructor
        """
        super(LinearActivator, self).__init__(minActivation, maxActivation, name)
        self._zeroActivationValue = float(zeroActivationValue) # Activation raises linearly between this value and _fullActivationValue (it remains 0 until here))
        self._fullActivationValue = float(fullActivationValue) # This value (and other values further away from _threshold in this direction) means total activation

    def computeActivation(self, normalizedValue):
        assert isinstance(normalizedValue, int) or isinstance(normalizedValue, float)
        value_range = self.valueRange
        assert value_range != 0

        raw_activation = (normalizedValue - self._zeroActivationValue) / value_range

        activation = raw_activation * self.activationRange + self._minActivation
        return sorted((self._minActivation, activation, self._maxActivation))[1] # clamp to activation range

    def getDirection(self):
        return 1 if self._fullActivationValue > self._zeroActivationValue else -1

    def getSensorWish(self, normalizedValue):
        assert isinstance(normalizedValue, int) or isinstance(normalizedValue, float)

        assert self.valueRange != 0

        if self.getDirection() > 0:
            # return how much is missing clamped to [0, 1]
            wish_value = sorted((0.0, (self._fullActivationValue - normalizedValue) / self.valueRange, 1.0))[1]
        else:
            # return how much is there more than desired clamped to [-1, 0]
            wish_value = sorted((-1.0, (self._fullActivationValue - normalizedValue) / abs(self.valueRange), 0.0))[1]

        return wish_value

    def _calculate_satisfaction_bound(self, satisfaction_threshold):
        """
        Calculates the activator specific activation threshold
        :param behaviour satisfaction_threshold, range [0,1]
        :return: a satisfaction bound in relation to the activator bounds
        """
        if satisfaction_threshold < 0 or satisfaction_threshold > 1:
            raise ValueError("satisfaction_threshold is " + str(satisfaction_threshold) + " and not in [0,1]")

        return self._zeroActivationValue + (self.valueRange * satisfaction_threshold)

    def getSensorPreconditionPDDL(self, sensorName, satisfaction_threshold, current_sensor_value):
        functionName = self.getPDDLFunctionName(sensorName)
        satisfaction_bound = self._calculate_satisfaction_bound(satisfaction_threshold)
        return PDDL(statement="( >= (" + functionName + ") {0:f} )".format(satisfaction_bound) if self.getDirection() == 1 else "( <= (" + functionName + ") {0:f} )".format(satisfaction_bound), functions = functionName)

    def getSensorStatePDDL(self, sensorName, normalizedValue, sensor_update_time):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="( = (" + functionName + ") {0:f} )".format(normalizedValue), functions=functionName,
                    time_stamp=sensor_update_time)

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


class EqualActivator(Activator):
    """
    This class is an activator that compares an arbitrary number sensor to a desired value
    """

    def __init__(self, desiredValue, minActivation=0, maxActivation=1, name=None):
        """
        Constructor
        """
        super(EqualActivator, self).__init__(minActivation=minActivation, maxActivation=maxActivation, name=name)

        self._desired = desiredValue

    def computeActivation(self, normalizedValue):
        return self._maxActivation if normalizedValue == self._desired else self._minActivation

    def getDirection(self):
        return 1.0

    def getSensorWish(self, normalizedValue):
        if normalizedValue == self._desired:
            return 0.0
        else:
            return 1.0

    def getSensorPreconditionPDDL(self, sensorName, satisfaction_threshold, current_sensor_value):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="( = (" + functionName + ") {0} )".format(self._desired),  functions=functionName)

    def getSensorStatePDDL(self, sensorName, normalizedValue, sensor_update_time):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="( = (" + functionName + ") {0} )".format(normalizedValue), functions=functionName,
                    time_stamp=sensor_update_time)

    def __str__(self):
        return "Equal Activator [{0} - {1}] ({2})".format(self._minActivation, self._maxActivation, self._desired)

    def __repr__(self):
        return "Equal Activator"


class StringActivator(EqualActivator):
    """
    This class is an activator that compares a String to a desired value. Respectively it handles all input values
    as strings

    In PDDL it is used like a boolean fact, hence a related effect would have to be defined in the same way,
    indicator +1 changes the sensor to the desired value -1 changes to another arbitrary value

    The implementation is a mix of the EqualActivator and the BooleanActivator

    """

    def __init__(self, desiredValue, minActivation=0, maxActivation=1, name=None):
        """
        Constructor
        """
        super(StringActivator, self).__init__(minActivation=minActivation, maxActivation=maxActivation, name=name,
                                              desiredValue=str(desiredValue))

    def computeActivation(self, normalizedValue):
        value = str(normalizedValue)
        return super(StringActivator, self).computeActivation(normalizedValue=value)

    def getSensorWish(self, normalizedValue):
        value = str(normalizedValue)
        return super(StringActivator, self).getSensorWish(normalizedValue=value)

    def getSensorPreconditionPDDL(self, sensorName, satisfaction_threshold, current_sensor_value):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="(" + functionName + ")", predicates=functionName)

    def getSensorStatePDDL(self, sensorName, normalizedValue, sensor_update_time):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="(" + functionName + ")" if normalizedValue == self._desired else "(not (" + functionName
                    + "))", predicates=functionName, time_stamp=sensor_update_time)

    def __str__(self):
        return "String Activator [{0} - {1}] ({2})".format(self._minActivation, self._maxActivation, self._desired)

    def __repr__(self):
        return "String Activator"
