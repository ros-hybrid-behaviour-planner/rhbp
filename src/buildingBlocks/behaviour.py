'''
Created on 13.04.2015

@author: stephan
''' 
import operator
import conditions
import warnings

class Behaviour(object):
    '''
    This is the smallest entity of an action.
    It may take a while or finish immediately.
    '''
    
    _instanceCounter = 0 # static counter to get distinguishable names

    def __init__(self, name = None):
        '''
        Constructor
        '''
        self._name = name if name else "Behaviour {0}".format(Behaviour._instanceCounter)
        self._preconditions = []
        self._isExecuting = False           # set this to True if this behaviour is selected for execution
        self._readyThreshold = 0.9 # This is the threshold that the preconditions must reach in order for this behaviour to be executable.  
        Behaviour._instanceCounter += 1
        
    def addPrecondition(self, precondition):
        '''
        This method adds an precondition to the behaviour.
        '''
        if isinstance(precondition, conditions.Condition):
            self._preconditions.append(precondition)
        else:
            warnings.warn("That's no condition!")
        
    
    def _getPreconditionActivation(self):
        '''
        This method should return the overall activation from all preconditions.
        In the easiest case this is equivalent to the product of the individual activations.
        '''
        return reduce(operator.mul, (x.activation for x in self._preconditions), 1)
    
    @property
    def readyThreshold(self):
        return self._readyThreshold
    
    @readyThreshold.setter
    def readyThreshold(self, threshold):
        self._readyThreshold = float(threshold)
    
    @property
    def preconditions(self):
        return self._preconditions
    
    @property
    def executable(self):
        return self._getPreconditionActivation() >= self._readyThreshold
    
    def __str__(self):
        return self._name
    
    def __repr__(self):
        return str(self)
        