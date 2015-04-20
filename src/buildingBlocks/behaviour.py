'''
Created on 13.04.2015

@author: stephan
''' 
import operator

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
        self._activators = []
        self._predecessors = []   # maybe unnecessary
        self._successors = []     # maybe unnecessary
        self._isExecuting = False # set this to True if this behaviour is selected for execution
        Behaviour._instanceCounter += 1
        
    def addActivator(self, activator):
        '''
        This method adds an activator to the behaviour.
        '''
        self._activators.append(activator)
    
    def getActivation(self):
        '''
        This method should return the overall activation from all dependencies.
        In the easiest case this is equivalent to the product of the individual Activations.
        '''
        return reduce(operator.mul, (x.getActivation() for x in self._activators), 1)
    
    def __str__(self):
        return self._name
        