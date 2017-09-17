## The Activators module
#Created on 22.04.2015
#@author: wypler, hrabia, rieger

from __future__ import division # force floating point division when using plain /
from behaviour_components.pddl import PDDL, get_pddl_effect_name
from std_msgs.msg import Float32
import rospy
import traceback

from .conditions import Conditonal
from .condition_elements import Wish

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.conditions')

class Condition(Conditonal):
    '''
    This is the basic Condition class it brings together the sensor and the activation function and takes care
    of the processing and caching of calculated data
    '''
    _instanceCounter = 0 # static _instanceCounter to get distinguishable names

    def __init__(self, sensor, activator, name = None, optional = False):
        """
        Constructor
        :param sensor: The sensor :py:class:`sensors.Sensor` that is evaluated by this condition
        :param activator: The :py:class:`Activator` that is used to calculate the activation from the sensor value
        :param name: condition name, a name will be generated if None is passed
        :param optional: If true the condition will not be considered for precondition satisfaction and only for activation calculation
        """
        super(Condition, self).__init__()
        self._name = name if name else "Condition{0}".format(Condition._instanceCounter)
        self._sensor = sensor
        self._activator = activator
        self._optional = optional

        self._normalizedSensorValue = 0
        self._satisfaction = 0

        Condition._instanceCounter += 1

    def sync(self):
        self._sensor.sync()

    def updateComputation(self):
        '''
        This method needs to be executed at first on every decision cycle before accessing all other values/results/methods
        '''
        try:
            self._normalizedSensorValue = self._normalize()
        except Exception as e:
            rhbplog.logwarn("Normalization failed: " + e.message)
            self._satisfaction = 0.0
            return

        try:
            self._satisfaction = self._activator.computeActivation(self._normalizedSensorValue)
        except AssertionError:
            rhbplog.logwarn("Wrong data type for %s in %s. Got %s. Possibly uninitialized%s sensor %s?. %s", self._sensor,
                          self._name, type(self._sensor.value), " optional" if self._sensor.optional else "",
                          self._sensor.name, traceback.format_exc())
            self._satisfaction = 0.0
            return

    def _normalize(self):
        '''
        Normalize the current sensor value into a floating point number
        '''
        if self._sensor:
            return self._sensor.value
        else:
            raise Exception("Sensor not available")

    def getDirections(self):
        return {self._get_pddl_effect_name(self._sensor): self._activator.getDirection()}

    def _get_pddl_effect_name(self, sensor):
        return self._activator.getPDDLFunctionName(sensor.name)

    def getWishes(self):
        '''
        returns a list of wishes (a wish is a tuple (effect name, indicator <float> [-1, 1]).
        Well, there is only one wish from one sensor - activator pair here but to keep a uniform interface with conjunction and disjunction this method wraps them into a list.
        '''
        try:
            indicator = self._activator.getSensorWish(self._normalizedSensorValue)
            return [Wish(sensor_name=self._sensor.name, indicator=indicator, activator_name=self._activator.name)]
        except AssertionError:
            rhbplog.logwarn("Wrong data type for %s in %s. Got %s. Possibly uninitialized%s sensor %s?", self._sensor, self._name, type(self._sensor.value), " optional" if self._sensor.optional else "", self._sensor.name)
            raise

    def _get_current_sensor_value_for_pddl_creation(self):
        return self._normalizedSensorValue

    def getPreconditionPDDL(self, satisfaction_threshold):
        return self._activator.get_sensor_precondition_pddl_using_current_value(self._sensor.name, satisfaction_threshold,
                                                         self._get_current_sensor_value_for_pddl_creation())

    def getStatePDDL(self):
        return [self._activator.getSensorStatePDDL(self._sensor.name, self._normalizedSensorValue)]

    def getFunctionNames(self):
        """
        Provides a list of virtual sensor activator function names
        :return list of function namestrings
        """
        return [self._get_pddl_effect_name(self._sensor)]

    @property
    def satisfaction(self):
        '''
        This property specifies to what extend the condition is fulfilled.
        '''
        return self._satisfaction

    @property
    def sensor(self):
        '''
        :return: The used sensor of this condition
        '''
        return self._sensor

    @property
    def activator(self):
        """
        :return: The used activator of this condition
        """
        return self._activator

    @property
    def optional(self):
        """
        The condition is either optional if itself is defined is optional or if the used sensor is optional
        :return: True if optional
        """
        return self._optional or self._sensor.optional

    @optional.setter
    def optional(self, value):
        self._optional = value

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

    def __init__(self, sensors, activator, name = None, optional = False):
        '''
        Constructor
        :param sensors: list, tuple of :py:class:`sensors.Sensor` objects that this condition is using
        :param activator: see :py:class:`Condition`
        :param name: see :py:class:`Condition`
        :param optional: see :py:class:`Condition`
        '''
        self._name = name if name else "MultiSensorCondition{0}".format(MultiSensorCondition._instanceCounter)

        assert hasattr(sensors, '__getitem__'), "sensors is not a tuple or list"
        self._sensors = sensors
        self._activator = activator
        self._optional = optional

        self._normalizedSensorValues = dict.fromkeys(self._sensors, 0)
        self._sensorSatisfactions = dict.fromkeys(self._sensors, 0)

        self._satisfaction = 0

        MultiSensorCondition._instanceCounter += 1


    def sync(self):
        for s in self._sensors:
            s.sync()

    def updateComputation(self):
        '''
        This method needs to be executed at first on every decision cycle before accessing all other values/results/methods
        '''
        try:
            self._normalize()

            self._computeSatisfactions()

            self._satisfaction = self._reduceSatisfaction()

        except Exception as e:
            rhbplog.logwarn("updateComputation failed: " +e.message)
            self._satisfaction = 0.0
            return

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
        ::return It has to return a new satisfaction value
        '''
        raise NotImplementedError()

    def getDirections(self):

        return {self._get_pddl_effect_name(sensor): self._activator.getDirection() for sensor in self._sensors}

    def getWishes(self):
        '''
        returns a list of wishes (a wish is a tuple (effect name, indicator <float> [-1, 1]).
        '''
        try:
            result = []
            for sensor in self._sensors:
                indicator = self._activator.getSensorWish(self._normalizedSensorValues[sensor])
                result.append(Wish(sensor_name=sensor.name, indicator=indicator, activator_name=self._activator.name))
            return result
        except AssertionError:
            rhbplog.logwarn("Wrong data type for %s in %s. Got %s. Possibly uninitialized%s sensor %s?", self._sensors, self._name, type(self._sensors.value), " optional" if self._sensors.optional else "", self._sensors.name)
            raise

    def _get_value_of_sensor_for_pddl_creation(self, sensor):
        return self._normalizedSensorValues[sensor]

    def getPreconditionPDDL(self, satisfaction_threshold):
        # Calling getSensorPreconditionPDDL for all sensors
        conditions = [self._activator.get_sensor_precondition_pddl_using_current_value(s.name, satisfaction_threshold,
                                                                self._get_value_of_sensor_for_pddl_creation(s)) for s in
                      self._sensors]

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

    def _get_pddl_effect_name(self, sensor):
        return self._activator.getPDDLFunctionName(sensor.name)

    def getFunctionNames(self):
        """
        Provides a list of virtual sensor activator function names
        :return list of function namestrings
        """
        return [self._get_pddl_effect_name(s) for s in self._sensors]

    @property
    def optional(self):
        return self._optional or reduce(lambda x, y: x and y, self._sensors, False)

    @optional.setter
    def optional(self, value):
        self._optional = value

    @property
    def sensors(self):
        return  self._sensors

    def __str__(self):
        return "{0}(satisfaction:{1})".format(self._name, self.satisfaction)

    def __repr__(self):
        return str(self)

class PublisherCondition(Condition):
    '''
    This is a extended condition that automatically publishes the normalized value
    of the sensor used by this condition
    '''

    def __init__(self, sensor, activator, name=None, optional=False):
        '''
        Constructor
        '''
        super(PublisherCondition, self).__init__(sensor=sensor, activator=activator, name=name, optional=optional)

        self._topic_name = activator.getPDDLFunctionName(sensorName=sensor.name)

        self.__pub = rospy.Publisher(self._topic_name, Float32, queue_size=10)

    def updateComputation(self):

        super(PublisherCondition, self).updateComputation()
        self.__pub.publish(self._normalizedSensorValue)

    @property
    def topic_name(self):
        return self._topic_name


class Activator(object):
    '''
    This is the abstract base class for an activator which reacts to a value according to the activation scheme implemented in it
    '''
    _instanceCounter = 0  # static _instanceCounter to get distinguishable names

    def __init__(self, minActivation=0, maxActivation=1, name=None):
        '''
        Constructor
        '''
        self._name = name if name else "Activator{0}".format(Activator._instanceCounter)
        self._minActivation = float(minActivation)
        self._maxActivation = float(maxActivation)
        Activator._instanceCounter += 1

    def getPDDLFunctionName(self, sensorName):
        return get_pddl_effect_name(sensorName, self._name)

    @staticmethod
    def restore_condition_name_from_pddl_function_name(pddl_function_name, sensor_name):
        return pddl_function_name[:(len(pddl_function_name) - len(sensor_name) - 1)]

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
        """
        Provide information if an increase or decrease of the sensor value is the intended direction for increasing the satisfaction
        :return: +1 for increase and -1 for decrease
        """
        raise NotImplementedError()

    def getSensorWish(self, normalizedValue):
        '''
        This method should return an indicator (float in range [-1, 1]) how much and in what direction the value should change in order to reach more activation.
        1 means the value should become true or increase, 0 it should stay the same and -1 it should decrease or become false.
        '''
        raise NotImplementedError()

    def get_sensor_precondition_pddl_using_current_value(self, sensor_name, satisfaction_threshold, current_sensor_value):
        '''
        see getSensorPreconditionPDDL(self, sensorName, satisfaction_threshold)
        :param current_sensor_value: current value of the sensor
               (in default condition implementations the normalized value)
        '''
        return self.getSensorPreconditionPDDL(sensorName=sensor_name, satisfaction_threshold=satisfaction_threshold)

    def getSensorPreconditionPDDL(self, sensorName, satisfaction_threshold):
        '''
        This method should produce valid PDDL condition expressions suitable for FastDownward (http://www.fast-downward.org/PddlSupport)
        :param sensorName: name of the considered sensor
        :param satisfaction_threshold: threshold value when the conditions becomes valid(true), this hint is not necessarily used, it depends on the activator function, range [0,1]
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

    @property
    def name(self):
        return self._name
    

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
        return 1 if self._desired else -1

    def getSensorWish(self, normalizedValue):
        assert isinstance(normalizedValue, bool)
        if normalizedValue == self._desired:
            return 0.0
        if self._desired == True:
            return 1.0
        return -1.0
    
    def getSensorPreconditionPDDL(self, sensorName, satisfaction_threshold):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement = "(" + functionName + ")" if self._desired == True else "(not (" + functionName + "))", predicates=functionName)

    def getSensorStatePDDL(self, sensorName, normalizedValue):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="(" + functionName + ")" if normalizedValue == True else "(not (" + functionName + "))", predicates=functionName)
        
    def __str__(self):
        return "Boolean Activator [{0} - {1}] ({2})".format(self._minActivation, self._maxActivation, self._desired)
    
    def __repr__(self):
        return "Boolean Activator"
    

class ThresholdActivator(Activator):
    '''
    This class is an activator that compares a sensor's value (expected to be int or float) to a threshold.
    '''
    def __init__(self, thresholdValue, isMinimum = True, valueRange = None, minActivation = 0, maxActivation = 1, name = None):
        """
        :param thresholdValue: This is the threshold Value above/below we get full activation depending on isMinimum
        :param isMinimum: isMinimum=True -> full activation above threshold, isMinimum=False -> full activation below threshold
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

    def getSensorPreconditionPDDL(self, sensorName, satisfaction_threshold):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement = "( >= (" + functionName + ") {0:f} )".format(self._threshold) if self.getDirection() == 1 else "( <= (" + functionName + ") {0:f} )".format(self._threshold), functions = functionName)
    
    def getSensorStatePDDL(self, sensorName, normalizedValue):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="( = (" + functionName + ") {0:f} )".format(normalizedValue), functions=functionName)
    
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
        :param step_size: value, bigger than 0
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
            return 0
        else:
            return 1

    def getDirection(self):
        return 1 if self.__maximize else -1

    def getSensorWish(self, normalizedValue):
        return self.getDirection()

    def getSensorPreconditionPDDL(self, sensorName, satisfaction_threshold):
        raise RuntimeError('use get_sensor_precondition_pddl_using_current_value')

    def get_sensor_precondition_pddl_using_current_value(self, sensor_name, satisfaction_threshold, current_value):
        function_name = self.getPDDLFunctionName(sensor_name)
        next_threshold = current_value + self.__step_size * self.getDirection()
        operator = '>=' if self.getDirection() > 0 else '<='
        statement = '( {0} ({1}) {2:f})'.format(operator,function_name,next_threshold)
        return PDDL(statement=statement, functions=function_name)

    def getSensorStatePDDL(self, sensorName, normalizedValue):
        function_name = self.getPDDLFunctionName(sensorName)
        return PDDL(statement="( = (" + function_name + ") {0:f} )".format(normalizedValue), functions=function_name)

    def __str__(self):
        return "Greedy Activator{0}".format("+" if self.__maximize else "-")

    def __repr__(self):
        return "Greedy Activator"


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
        '''
        Calculates the activator specific activation threshold
        :param behaviour satisfaction_threshold, range [0,1]
        :return: a satisfaction bound in relation to the activator bounds
        '''
        if satisfaction_threshold < 0 or satisfaction_threshold > 1:
            raise ValueError("satisfaction_threshold is " + str(satisfaction_threshold) + " and not in [0,1]")

        return self._zeroActivationValue + (self.valueRange * satisfaction_threshold)

    def getSensorPreconditionPDDL(self, sensorName, satisfaction_threshold):
        functionName = self.getPDDLFunctionName(sensorName)
        satisfaction_bound = self._calculate_satisfaction_bound(satisfaction_threshold)
        return PDDL(statement = "( >= (" + functionName + ") {0:f} )".format(satisfaction_bound) if self.getDirection() == 1 else "( <= (" + functionName + ") {0:f} )".format(satisfaction_bound), functions = functionName)

    def getSensorStatePDDL(self, sensorName, normalizedValue):
        functionName = self.getPDDLFunctionName(sensorName)
        return PDDL(statement = "( = (" + functionName + ") {0:f} )".format(normalizedValue), functions = functionName)

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

class StringActivator(BooleanActivator):
    '''
    This class is an activator that compares a String to a desired value
    '''

    def __init__(self, desiredValue, minActivation=0, maxActivation=1, name=None):
        '''
        Constructor
        '''
        super(StringActivator, self).__init__(desiredValue=desiredValue, minActivation=minActivation, maxActivation=maxActivation, name=name)

    def computeActivation(self, normalizedValue):
        value = str(normalizedValue)
        assert isinstance(value, str)
        return self._maxActivation if value == self._desired else self._minActivation

    def getDirection(self):
        return 1.0

    def getSensorWish(self, normalizedValue):
        value = str(normalizedValue)
        assert isinstance(value, str)
        if value == self._desired:
            return 0.0
        else:
            return 1.0

    def __str__(self):
        return "String Activator [{0} - {1}] ({2})".format(self._minActivation, self._maxActivation, self._desired)

    def __repr__(self):
        return "String Activator"

