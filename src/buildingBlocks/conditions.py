'''
Created on 13.04.2015

@author: stephan
'''

import activators
import warnings

class Conditonal(object):
    '''
    This is the base class for conditions or anything that spreads activations.
    Subclasses have to provide the activation property
    '''
    _instanceCounter = 0 # static _instanceCounter to get distinguishable names
    
    def __init__(self):
        Conditonal._instanceCounter += 1
        
    @property
    def activation(self):
        '''
        This property means fulfillment.
        Note that the opposite (1 - activation) is a good measure of the effort that is necessary to reach fulfillment - so it can be used as activation induced in behaviours to reach fulfillment.
        '''
        raise NotImplementedError()
    
    def __str__(self):
        return "Conditional"
    
    def __repr__(self):
        return str(self)


class Condition(Conditonal):
    '''
    This class wraps a sensor and an activator to build a precondition
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
    
    @property
    def activation(self):
        '''
        This property specifies to what extend the condition is fulfilled.
        Note that the opposite (1 - activation) is a good measure of the effort that is necessary to satisfy this condition - so it can be used as activation induced in behaviours that change the sensor value in the satisfactory direction.
        '''
        return self._activator.computeActivation(self._sensor.value)
    
    def __str__(self):
        return "{0} wrapping {1} with {2}: {3}".format(self._name, self._sensor, self._activator, self.activation)
    
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
    def activation(self):
        '''
        The disjunction of activations is the highest activation.
        '''
        return max((x.activation for x in self._conditions))
        
    @property
    def conditions(self):
        return self._conditions
    
    def __str__(self):
        return "{0} made of: {1}: {2}".format(self._name, self._conditions, self.activation)
    
    def __repr__(self):
        return str(self)
