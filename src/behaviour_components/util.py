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
    def __init__(self, statement = None, predicates = None, functions = None):
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
        if functions is not None:
            if type(functions) is list:
                self.functions = set(functions)
            else:
                self.functions = set([functions])
        else:
            self.functions = set()
    
    def __repr__(self):
        return "PDDL: predicates: " + " ".join(self.predicates) + " functions: " + " ".join(self.functions) + " statement: " + self.statement

class Effect(object):
    """
    This class models effects and their combinations.
    All effects (correlations) are assumed to happen simultaneously except otherwise stated.
    """
    def __init__(self, sensorName, indicator, sensorType = bool, realWorldImpact = 1.0, condition = None):
        self.sensorName = sensorName
        self.indicator = indicator
        self.sensorType = sensorType
        self.realWorldImpact = realWorldImpact
        self.condition = condition
        
    def getPDDL(self):
        pddl = PDDL(statement = "(")
        obr = 1 # count opened brackets
        if self.condition is not None:
            pddl.statement += "when ({0}) (".format(self.condition)
            obr += 1
        if self.sensorType is bool:
            pddl.statement += self.sensorName if self.indicator > 0 else "not ({0})".format(self.sensorName)
            pddl.predicates.add(self.sensorName) # TODO: What about other predicates employed in conditions by user??
        else: # its numeric and not bool
            pddl.statement += "{0} ({1}) {2}".format("increase" if self.indicator > 0.0 else "decrease", self.sensorName, self.realWorldImpact)
            pddl.functions.add(self.sensorName) # TODO: What about other functions employed in conditions by user??
        pddl.statement += ")" * obr # close the brackets
        return pddl
        
        
        
        
        
        
        
        