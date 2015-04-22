'''
Created on 22.04.2015

@author: stephan
'''

import operator
import conditions
import warnings

class Goal(object): #TODO: Although it is similar to Preconditions of a behaviour we accept the code duplication for now
    '''
    This class represents a goal. Goals have conditions that need to be fulfilled.
    '''
    
    _instanceCounter = 0

    def __init__(self, name = None):
        '''
        Constructor
        '''
        self._name = name if name else "Goal {0}".format(Goal._instanceCounter)
        self._conditions = []
        Goal._instanceCounter += 1
        
    def addCondition(self, condition):
        '''
        This method adds an precondition to the behaviour.
        '''
        if isinstance(condition, conditions.Condition):
            self._conditions.append(condition)
        else:
            warnings.warn("That's no condition!")
    
    @property
    def statisfaction(self):
        '''
        This method should return the overall activation from all conditions.
        In the easiest case this is equivalent to the product of the individual activations.
        '''
        return reduce(operator.mul, (x.activation for x in self._conditions), 1)
    
    @property
    def conditions(self):
        return self._conditions
    
    def __str__(self):
        return "{0} with the following conditions:\n{1}".format(self._name, "\n".join([str(x) for x in self._conditions]))
    
    def __repr__(self):
        return self._name