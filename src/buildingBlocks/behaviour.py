'''
Created on 13.04.2015

@author: stephan
''' 
import operator
import conditions
import warnings
import itertools

class Behaviour(object):
    '''
    This is the smallest entity of an action.
    It may take a while or finish immediately.
    '''
    
    _instanceCounter = 0 # static counter to get distinguishable names

    def __init__(self, name = None, **kwargs):
        '''
        Constructor
        '''
        self._name = name if name else "Behaviour {0}".format(Behaviour._instanceCounter)
        self._preconditions = []
        self._isExecuting = False  # Set this to True if this behaviour is selected for execution.
        self._readyThreshold = kwargs["readyThreshold"] if "readyThreshold" in kwargs else 0.8 # This is the threshold that the preconditions must reach in order for this behaviour to be executable.
        self._correlations = kwargs["correlations"] if "correlations" in kwargs else {}        # Stores sensor correlations in dict in form: sensor <Sensor> : correlation <float> [-1 to 1]. 1 Means high positive correlation to the value or makes it become True, -1 the opposite and 0 does not affect anything.
        self._predecessors = []    # List of predecessors of this behaviour. This list is refreshed each iteration based on correlations.
        self._successors = []      # List of successors. This list is refreshed each iteration based on correlations.
        self._conflictors = []     # List of successors. This list is refreshed each iteration based on correlations.
        self._manager = None       # This is the Manager that supplies global variables an access to all foreign objects
        self._activation = 0.0     # This is the magic activation that it's all about
        Behaviour._instanceCounter += 1
        
    def addPrecondition(self, precondition):
        '''
        This method adds an precondition to the behaviour.
        There is an AND relationship between all elenents (all have to be fulfilled so that the behaviour is ready)
        To enable OR behaviour use the pseudo Conditional Disjunction.
        '''
        if issubclass(type(precondition), conditions.Conditonal):
            self._preconditions.append(precondition)
        else:
            warnings.warn("That's no conditional object!")
    
    def getPreconditionSatisfaction(self):
        '''
        This method should return the overall activation from all preconditions.
        In the easiest case this is equivalent to the product of the individual activations.
        '''
        return reduce(operator.mul, (x.satisfaction for x in self._preconditions), 1)
    
    def getWishes(self):
        '''
        This method returns a dict of wishes.
        For all sensors wrapped in conditions for this goal this dict says what changes are necessary to achieve it
        '''
        return dict(itertools.chain.from_iterable([x.getWishes() for x in self._preconditions]))
    
    def getActivationFromPreconditions(self):
        '''
        This method computes the activation from the situation.
        It is the average satisfaction of preconditions.
        '''
        return reduce(lambda x, y: x + y, (x.satisfaction for x in self._preconditions)) / len(self._preconditions)
    
    def getActivationFromGoals(self):
        '''
        This method computes the activation from goals.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        '''
        pass
    
    @property
    def correllations(self):
        return self._correlations
    
    @correllations.setter
    def correllations(self, correllations):
        '''
        This is for initializing the correlations.
        correlations must be a dict of form sensor <Sensor> : correlation <float> [-1 to 1].
        '''
        self._correlations = correllations
    
    @property
    def activation(self):
        return self._activation
    
    @property
    def readyThreshold(self):
        return self._readyThreshold
    
    @readyThreshold.setter
    def readyThreshold(self, threshold):
        self._readyThreshold = float(threshold)
        
    @property
    def manager(self):
        return self._manager
    
    @manager.setter
    def manager(self, manager):
        self._manager = manager
    
    @property
    def preconditions(self):
        return self._preconditions
        
    @property
    def executable(self):
        return self.getPreconditionSatisfaction() >= self._readyThreshold
    
    @property
    def name(self):
        return self._name
    
    @name.setter
    def name(self, newName):
        self._name = newName
    
    def __str__(self):
        return "{0} with the following preconditions:\n{1}".format(self._name, "\n".join([str(x) for x in self._preconditions]))
    
    def __repr__(self):
        return self._name
        