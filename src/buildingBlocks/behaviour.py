'''
Created on 13.04.2015

@author: stephan
''' 
import operator

class behaviour(object):
    '''
    This is the smallest entity of an action.
    It may take a while or finish immediately.
    '''
    _activators = []
    _predecessors = []
    _successors = []
    _isExecuting = False
    _name = ""


    def __init__(self, name):
        '''
        Constructor
        '''
        self._name = name
        
    def addDependency(self, activator):
        self._activators.append(activator)
    
    def getActivation(self):
        '''
        This method should return the overall activation from all dependencies.
        In the easiest case this is equivalent to the product of the individual Activations.
        '''
        return reduce(operator.mul, (x.getActivation() for x in self._activators), 1)
        