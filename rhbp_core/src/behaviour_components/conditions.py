'''
Created on 13.04.2015

@author: stephan
'''

import warnings
import operator
import itertools
from util import PDDL

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
    
    def getPreconditionPDDL(self):
        '''
        This method should produce valid PDDL condition expressions suitable for FastDownward (http://www.fast-downward.org/PddlSupport)
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
    
    def __str__(self):
        return "Conditional"
    
    def __repr__(self):
        return str(self)
    
# THINK ABOUT ME:
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
    
    def addCondition(self, condition):
        '''
        This method adds an precondition to the disjunction.
        '''
        if isinstance(condition, Conditonal):
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
        '''
        returns a list of wishes (a wish is a tuple (sensor name <string>, indicator <float> [-1, 1]).
        Each component of the disjunction contributes its wish to the list.
        '''
        l = []
        for c in self._conditions:
            l = list(itertools.chain(l, c.getWishes()))
        return l
    
    def getPreconditionPDDL(self):
        pddl = PDDL(statement = "(or")
        for c in self._conditions:
            cond_pddl = c.getPreconditionPDDL()
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
    

class Conjunction(Conditonal):
    '''
    This class is a pseudo condition composed of two regular conditions. It performs an OR relation.
    '''
    
    def __init__(self, *args, **kwargs):
        '''
        Constructor
        '''
        super(Conjunction, self).__init__()
        self._name = kwargs["name"] if "name" in kwargs else "Conjunction {0}".format(Conditonal._instanceCounter)
        self._conditions = list(args) # TODO check types!
    
    def addCondition(self, condition):
        '''
        This method adds an precondition to the disjunction.
        '''
        if isinstance(condition, Conditonal):
            self._conditions.append(condition)
        else:
            warnings.warn("That's no condition!")
    
    @property
    def satisfaction(self):
        '''
        The conjuction of activations is the product of individual satisfactions.
        '''
        return reduce(operator.mul, (x.satisfaction for x in self._conditions), 1)
    
    def getWishes(self):
        '''
        returns a list of wishes (a wish is a tuple (sensor name <string>, indicator <float> [-1, 1]).
        Each component of the conjunction contributes its wish to the list.
        '''
        l = []
        for c in self._conditions:
            l = list(itertools.chain(l, c.getWishes()))
        return l
    
    def getPreconditionPDDL(self):
        pddl = PDDL(statement = "(and")
        for c in self._conditions:
            cond_pddl = c.getPreconditionPDDL()
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
        
    @property
    def satisfaction(self):
        '''
        The negated satisfaction
        '''
        return self._condition.satisfaction() * -1
    
    def getWishes(self):
        '''
        returns a list of wishes (a wish is a tuple (sensor name <string>, indicator <float> [-1, 1]).
        Each component of the conjunction contributes its wish to the list.
        '''
        wishes = self._condition.getWishes()
        
        l = []
        for w in wishes:
            l.append((w[0],w[1] * -1)) #negation
        return l
    
    def getPreconditionPDDL(self): 
        '''
        The negated precondition PDDL
        '''       
        pddl = self._condition.getPreconditionPDDL()
        pddl.statement = "(not {0})".format(pddl.statement)
        return pddl
    
    def getStatePDDL(self):
        return self._condition.getStatePDDL()
            
    @property
    def optional(self):
        return self._condition.optional
    
    def __str__(self):
        return "{0} of {1}".format(self._name, self._condition)
    
