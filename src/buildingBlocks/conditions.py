'''
Created on 13.04.2015

@author: stephan
'''

import activators
import warnings


class Conditonal(object):
    '''
    This is the base class for conditions or anything that spreads activations.
    Subclasses have to provide the satisfaction property
    '''
    _instanceCounter = 0 # static _instanceCounter to get distinguishable names
    
    def __init__(self):
        Conditonal._instanceCounter += 1
        
    @property
    def satisfaction(self):
        '''
        This property means fulfillment.
        '''
        raise NotImplementedError()
    
    def getWishes(self):
        '''
        This method should return a list of (sensor, indicator) tuples where the indicator should return the strength normalized strength and direction of sensor changes to make the condition True.
        Indicator value range should be between -1 and 1 where -1 means the value must decrease or become False, 0 means no change is necessary and should remain, 1 means the value should increase or become True.
        '''
        raise NotImplementedError()
        
    
    def __str__(self):
        return "Conditional"
    
    def __repr__(self):
        return str(self)


class Condition(Conditonal):
    '''
    This class wraps a sensor and an activator to build a condition and produce a satisfaction value
    '''
    _instanceCounter = 0 # static _instanceCounter to get distinguishable names

    def __init__(self, sensor, activator, name = None):
        '''
        Constructor
        '''
        super(Condition, self).__init__()
        self._name = name if name else "Condition {0}".format(Conditonal._instanceCounter)
        self._sensor = sensor
        self._activator = activator
        
    @property
    def activator(self):
        return self._activator
    
    @activator.setter
    def activator(self, newActivator):
        if issubclass(newActivator, activators.Activator):
            self._activator = newActivator
        else:
            warnings.warn("That's no activator!")
            
    def getWishes(self):
        return [(self._sensor, self._activator.getWish(self._sensor.value))]
    
    @property
    def satisfaction(self):
        '''
        This property specifies to what extend the condition is fulfilled.
        '''
        return self._activator.computeActivation(self._sensor.value)
    
    @property
    def sensor(self):
        return self._sensor
    
    def __str__(self):
        return "{0} {{{1} + {2}: {3}}}".format(self._name, self._sensor, self._activator, self.satisfaction)
    
    def __repr__(self):
        return str(self)


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
        Disjunction._instanceCounter += 1
    
    def addCondition(self, condition):
        '''
        This method adds an precondition to the disjunction.
        '''
        if isinstance(condition, Condition):
            self._conditions.append(condition)
        else:
            warnings.warn("That's no condition!")
    
    @property
    def satisfaction(self):
        '''
        The disjunction of activations is the highest satisfaction.
        '''
        return max((x.satisfaction for x in self._conditions))
    
    def getWishes(self):
        return [(c.sensor, c._activator.getWish(c.sensor.value)) for c in self._conditions]
        
    @property
    def conditions(self):
        return self._conditions
    
    def __str__(self):
        return "{0} made of: {1}: {2}".format(self._name, self._conditions, self.satisfaction)
    
    def __repr__(self):
        return str(self)
