'''
Created on 06.10.2015

@author: stephan
'''
from __future__ import division # force floating point division when using plain /
import operator
import itertools
import rospy

class PDDL(object):
    """
    This class wraps PDDL fragments while building statements. It is used to collect predicates that are used in the statement for easier domain description generation.
    """
    def __init__(self, statement = None, predicates = None):
        if statement is not None:
            self.statement = statement
        else:
            self.statement = ""
        if predicates is not None:
            if type(predicates) is list:
                self.predicates = set(predicates)
            else:
                self.predicates = set([predicates])
        else:
            self.predicates = set()

class Effect(object):
    """
    This class models effects and their combinations.
    All effects (correlations) are assumed to happen simultaneously except otherwise stated.
    """
    def __init__(self, mutuallyExclusiveWith):
        """
        TODO: implement me
        """
        raise NotImplementedError