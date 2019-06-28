'''
Created on 13.04.2015

@author: wypler, hrabia
'''
import traceback
import warnings
import operator
import itertools
import rospy

from std_msgs.msg import Float32

from behaviour_components.activators import rhbplog
from behaviour_components.pddl import get_pddl_effect_name
from behaviour_components.activators import BooleanActivator, GreedyActivator
from .pddl import PDDL
from .condition_elements import Wish

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.conditions')


class Conditonal(object):
    '''
    This is the base class for conditions or anything that spreads activations.
    Subclasses have to provide the satisfaction property
    '''
    _instanceCounter = 0 # static _instanceCounter to get distinguishable names
    
    def __init__(self):
        Conditonal._instanceCounter += 1
        self._last_update_step = -1

    def need_update(self, manager_step):
        """
        Check if in the current manager step updates have already been computed
        and set last update step to given manager_step
        :param manager_step: current planning step of the manager, pass none to force an update
        :return: True if sync and update_computation are required
        """
        if manager_step is None:
            return True
        if self._last_update_step != manager_step:
            self._last_update_step = manager_step
            return True
        else:
            return False

    def sync(self):
        '''
        Synchronize the current input state of all used components
        Should be called before updateComputation
        '''

        raise NotImplementedError()
        
    def updateComputation(self):
        '''
        This method is called once per step and should trigger complex computations that should be cached
        '''
        pass
        
    @property
    def satisfaction(self):
        '''
        This property means fulfillment.
        '''
        raise NotImplementedError()
    
    def getWishes(self):
        '''
        This method should return a list of Wish() objects where the indicator should return the strength normalized strength and direction of sensor changes to make the condition True.
        Indicator value range should be between -1 and 1 where -1 means the value must decrease or become False, 0 means no change is necessary and should remain, 1 means the value should increase or become True.
        '''
        raise NotImplementedError()

    def getDirections(self):
        """
        Provide information if an increase or decrease of sensor values is the intended direction for increasing the satisfaction
        :return: Dict of sensor, direction pairs  +1 for increase and -1 for decrease
        """
        raise NotImplementedError()
    
    def getPreconditionPDDL(self, satisfaction_threshold):
        '''
        This method should produce valid PDDL condition expressions suitable for FastDownward (http://www.fast-downward.org/PddlSupport)
        :param satisfaction_threshold: threshold value that specifies when a precondition is considered valid, range [0,1]
        :returns pddl.PDDL() object instance
        '''
        raise NotImplementedError()
    
    def getStatePDDL(self):
        '''
        This method should produce  a list of valid PDDL statements describing the (initial) state of the world suitable for FastDownward (http://www.fast-downward.org/PddlSupport)
         '''
        raise NotImplementedError()
    
    @property
    def optional(self):
        raise NotImplementedError()

    @property
    def conditions(self):
        raise NotImplementedError()
    
    def __str__(self):
        return "Conditional"
    
    def __repr__(self):
        return str(self)
    
# TODO THINK ABOUT ME:
# The next two classes make it possible to build arbitrary complex preconditions using logical AND and OR.
# This notion is visible for the preconditions only: IT IS TRANSPARENT FOR WISHES so the outside world will try to satisfy ALL given conditions even if they are actually ORed 
# Another issue are constructions containing optional sensors: what should I do if only the optional part fails?! I think this construct shall be mandatory if it contains at least one mandatory component


class Disjunction(Conditonal):
    '''
    This class is a pseudo condition composed of two regular conditions. It performs an OR relation.
    '''
    
    def __init__(self, *args, **kwargs):
        '''
        Constructor
        '''
        super(Disjunction, self).__init__()
        self._name = kwargs["name"] if "name" in kwargs else "Disjunction {0}".format(Conditonal._instanceCounter)
        self._conditions = list(args)
        self._satisfaction = 0

    def addCondition(self, condition):
        '''
        This method adds an precondition to the disjunction.
        '''
        if isinstance(condition, Conditonal):
            self._conditions.append(condition)
        else:
            warnings.warn("That's no condition!")

    def need_update(self, manager_step):
        """
        Check if in the current manager step updates have already been computed
        and set last update step to given manager_step
        :param manager_step: current planning step of the manager
        :return: True if sync and update_computation are required
        """
        need_update = False
        for c in self._conditions:
            if c.need_update(manager_step):
                need_update = True
                if manager_step is not None:
                    self._last_update_step = manager_step

        return need_update

    def sync(self):
        for c in self._conditions:
            c.sync()
            
    def updateComputation(self):
        for c in self._conditions:
            c.updateComputation()

        '''
        The disjunction of activations is the highest satisfaction.
        '''
        self._satisfaction = max((x.satisfaction for x in self._conditions))

    @property
    def satisfaction(self):
        return self._satisfaction

    def getDirections(self):
        d = {}

        for c in self._conditions:
            d.update(c.getDirections())

        return d

    def getWishes(self):
        '''
        returns a list of Wish() objects.
        Each component of the disjunction contributes its wish to the list.
        '''
        l = []
        for c in self._conditions:
            l = list(itertools.chain(l, c.getWishes()))
        return l
    
    def getPreconditionPDDL(self, satisfaction_threshold):
        pddl = PDDL(statement = "(or")
        for c in self._conditions:
            cond_pddl = c.getPreconditionPDDL(satisfaction_threshold)
            pddl.statement += " {0}".format(cond_pddl.statement)
            pddl.predicates = pddl.predicates.union(cond_pddl.predicates)
            pddl.functions = pddl.functions.union(cond_pddl.functions)
        pddl.statement += ")"
        return pddl
    
    def getStatePDDL(self):
        return list(itertools.chain.from_iterable([c.getStatePDDL() for c in self._conditions])) # this list may contain duplicates but the behaviour will take care of that.

    def getSatisfiedConditions(self, ready_threshold):
        '''
        return the list of all satisfied conditions of this disjunction based on the given ready_threshold
        the list is sorted by satisfaction value in decending order
        This allows a behaviour to check which precondition have been responsible for the activation
        '''
        satisfied_conditions = [con for con in self._conditions if con.satisfaction >= ready_threshold]

        return sorted(satisfied_conditions,key=lambda con:con.satisfaction,reverse=True)
        
    @property
    def conditions(self):
        return self._conditions
    
    @property
    def optional(self):
        return False if len(filter(lambda x: x.optional == False, self._conditions)) > 0 else True # A combination of conditions is mandatory if it has at least one mandatory component
    
    def __str__(self):
        return "{0} made of: {1}: {2}".format(self._name, self._conditions, self.satisfaction)
    

class Conjunction(Conditonal):
    '''
    This class is a pseudo condition composed of two regular conditions. It performs an AND relation.
    '''
    
    def __init__(self, *args, **kwargs):
        '''
        Constructor
        '''
        super(Conjunction, self).__init__()
        self._name = kwargs["name"] if "name" in kwargs else "Conjunction {0}".format(Conditonal._instanceCounter)
        self._conditions = list(args) # TODO check types!
        self._satisfaction = 0
    
    def addCondition(self, condition):
        '''
        This method adds an precondition to the disjunction.
        '''
        if isinstance(condition, Conditonal):
            self._conditions.append(condition)
        else:
            warnings.warn("That's no condition!")

    def need_update(self, manager_step):
        """
        Check if in the current manager step updates have already been computed
        and set last update step to given manager_step
        :param manager_step: current planning step of the manager
        :return: True if sync and update_computation are required
        """
        need_update = False
        for c in self._conditions:
            if c.need_update(manager_step):
                need_update = True
                if manager_step is not None:
                    self._last_update_step = manager_step

        return need_update

    def sync(self):
        for c in self._conditions:
            c.sync()

    def updateComputation(self):
        for c in self._conditions:
            c.updateComputation()

        '''
        The conjuction of activations is the product of individual satisfactions.
        '''
        self._satisfaction = reduce(operator.mul, (x.satisfaction for x in self._conditions), 1)
        # TODO getDirection and getWishes, getPreconditionPDDL could be cached too
    
    @property
    def satisfaction(self):
        return self._satisfaction

    def getDirections(self):
        d = {}

        for c in self._conditions:
            d.update(c.getDirections())

        return d
    
    def getWishes(self):
        '''
        returns a list of Wish() objects
        Each component of the conjunction contributes its wish to the list.
        '''
        l = []
        for c in self._conditions:
            l = list(itertools.chain(l, c.getWishes()))
        return l
    
    def getPreconditionPDDL(self, satisfaction_threshold):
        pddl = PDDL(statement = "(and")
        for c in self._conditions:
            cond_pddl = c.getPreconditionPDDL(satisfaction_threshold)
            pddl.statement += " {0}".format(cond_pddl.statement)
            pddl.predicates = pddl.predicates.union(cond_pddl.predicates)
            pddl.functions = pddl.functions.union(cond_pddl.functions)
        pddl.statement += ")"
        return pddl
    
    def getStatePDDL(self):
        return list(itertools.chain.from_iterable([c.getStatePDDL() for c in self._conditions])) # this list may contain duplicates but the behaviour will take care of that.
        
    @property
    def conditions(self):
        return self._conditions
    
    @property
    def optional(self):
        return False if len(filter(lambda x: x.optional == False, self._conditions)) > 0 else True # A combination of conditions is mandatory if it has at least one mandatory component
    
    def __str__(self):
        return "{0} made of: {1}: {2}".format(self._name, self._conditions, self.satisfaction)
        
    
class Negation(Conditonal):
    '''
    This class is a pseudo conditional composed of another conditional that is going to be negated
    '''
    
    def __init__(self, conditional):
        '''
        Constructor
        '''
        super(Negation, self).__init__()
        self._name = "Negation {0}".format(Conditonal._instanceCounter)
        assert isinstance(conditional, Conditonal), "Only Conditionals can be negated"
        self._condition = conditional
        self._name = "not_"+self._condition._name

    def sync(self):
        self._condition.sync()

    def updateComputation(self):
        self._condition.updateComputation()
        #TODO getDirection and getWishes, getPreconditionPDDL could be cached too
        
    @property
    def satisfaction(self):
        '''
        The negated satisfaction
        '''
        return 1 - self._condition.satisfaction

    def getDirections(self):
        d = {}
        # invert all directions
        for sensor, direction in self._condition.getDirections().iteritems():
            d[sensor] = direction * -1

        return d
    
    def getWishes(self):
        '''
        returns a list of Wish() objects.
        Each component of the conjunction contributes its wish to the list.
        '''
        wishes = self._condition.getWishes()
        directions = self.getDirections()
        
        l = []
        for w in wishes:
            wish_value = w.indicator
            if wish_value > 0:
                wish_value = (1 - wish_value) * -1
            elif wish_value < 0:
                wish_value = (1 - abs(wish_value))
            else: #wish value == 0
                # determine if it has to be -1 or +1
                wish_value = directions[w.get_pddl_effect_name()]
            l.append(Wish(sensor_name=w.sensor_name, indicator=wish_value, activator_name=w.activator_name)) #negation
        return l

    def getPreconditionPDDL(self, satisfaction_threshold):
        '''
        The negated precondition PDDL
        '''       
        pddl = self._condition.getPreconditionPDDL(satisfaction_threshold)
        pddl.statement = "(not {0})".format(pddl.statement)
        return pddl
    
    def getStatePDDL(self):
        return self._condition.getStatePDDL()
            
    @property
    def optional(self):
        return self._condition.optional

    @property
    def conditions(self):
        return [self._condition]
    
    def __str__(self):
        return "{0} of {1}".format(self._name, self._condition)


class Condition(Conditonal):
    '''
    This is the basic Condition class it brings together the sensor and the activation function and takes care
    of the processing and caching of calculated data
    '''
    _instanceCounter = 0 # static _instanceCounter to get distinguishable names

    def __init__(self, sensor, activator, name=None, optional=False, optional_for_planning=False):
        """
        Constructor
        :param sensor: The sensor :py:class:`sensors.Sensor` that is evaluated by this condition
        :param activator: The :py:class:`Activator` that is used to calculate the activation from the sensor value
        :param name: condition name, a name will be generated if None is passed
        :param optional: If true the condition will not be considered for precondition satisfaction and only for
                         activation calculation
        :param optional_for_planning: Ignore this condition in symbolic planning. This makes sense in cases where there
                                      is no way for the planner/behaviours to fulfil the condition, e.g. a condition
                                      that relies on the number of registered goals.
        """
        super(Condition, self).__init__()
        self._name = name if name else "Condition{0}".format(Condition._instanceCounter)
        self._sensor = sensor
        self._activator = activator
        self._optional = optional
        self._optional_for_planning = optional_for_planning

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
            rhbplog.logwarn("updateComputation:Wrong data type for %s in %s. Got type:'%s' and value '%s'. Possibly uninitialized%s sensor %s?. %s", self._sensor,
                          self._name, type(self._sensor.value), str(self._sensor.value), " optional" if self._sensor.optional else "",
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
        return {get_pddl_effect_name(self._sensor.name, self._activator.name): self._activator.getDirection()}

    def getWishes(self):
        '''
        returns a list of wishes (a wish is a tuple (effect name, indicator <float> [-1, 1]).
        Well, there is only one wish from one sensor - activator pair here but to keep a uniform interface with
        conjunction and disjunction this method wraps them into a list.
        '''
        try:
            indicator = self._activator.getSensorWish(self._normalizedSensorValue)
            return [Wish(sensor_name=self._sensor.name, indicator=indicator, activator_name=self._activator.name)]
        except AssertionError:
            rhbplog.logerr("getWishes:Wrong data type for %s in %s with %s of type %s. Type:'%s', Value '%s' "
                           "nomalized value '%s'. Possibly uninitialized %s sensor %s?", self._sensor,
                           self._name, self._activator.name, type(self._activator), type(self._sensor.value),
                           str(self._sensor.value), str(self._normalizedSensorValue),
                           " optional" if self._sensor.optional else "", self._sensor.name)
            raise

    def _get_current_sensor_value_for_pddl_creation(self):
        return self._normalizedSensorValue

    def getPreconditionPDDL(self, satisfaction_threshold):

        if self._optional_for_planning:
            return PDDL()
        else:
            return self._activator.getSensorPreconditionPDDL(self._sensor.name, satisfaction_threshold,
                                                         self._get_current_sensor_value_for_pddl_creation())

    def getStatePDDL(self):
        return [self._activator.getSensorStatePDDL(self._sensor.name, self._normalizedSensorValue,
                                                   self._sensor.value_update_time)]

    def getFunctionNames(self):
        """
        Provides a list of virtual sensor activator function names
        :return list of function namestrings
        """
        return [get_pddl_effect_name(self._sensor.name, self._activator.name)]

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


class PublisherCondition(Condition):

    '''
    This is a extended condition that automatically publishes the satisfaction value of the condition
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
        self.__pub.publish(self.satisfaction)

    @property
    def topic_name(self):
        return self._topic_name


def create_condition_from_effect(effect, sensor):
    if effect.sensor_type == str(bool):
        desired_value = True if effect.indicator > 0 else False
        activator = BooleanActivator(desiredValue=desired_value)
        return Condition(activator=activator, sensor=sensor)

    if effect.sensor_type == str(int) or effect.sensor_type == str(float):
        activator = GreedyActivator(maximize=effect.indicator > 0, step_size=abs(effect.indicator))
        return Condition(activator=activator, sensor=sensor)

    raise RuntimeError(msg='Cant create condition for effect type \'' + effect.sensor_type + '\'. Overwrite the method Effect.create_condition_from_effect to handle the type')
