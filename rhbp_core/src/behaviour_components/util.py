'''
Created on 06.10.2015

@author: stephan
'''
from __future__ import division # force floating point division when using plain /
import re
import rospy

functionRegex = re.compile(r'\s*\(\s*=\s*\(\s*([a-zA-Z0-9_\.-]+)\s*\)\s*([-+]?[0-9]*\.?[0-9]+)\s*\)\s*')
predicateRegex = re.compile(r'\s*\(\s*((not)\s*\()?\s*([a-zA-Z0-9_\.-]+)\s*\)?\s*\)\s*')

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
        
    def getEffectPDDL(self):
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
        
        
def tokenizePDDL(pddlString):
    '''
    This function returns a list of first level tokens.
    Tokens are enclosed in braces.
    '''
    if not re.search(r'\(', pddlString):
        return []
    tokens = []
    snippet = ""
    obr = 0
    for c in pddlString:
        snippet += c
        if c == '(':
            obr += 1
        elif c == ')':
            obr -= 1
            if obr == 0:
                tokens.append(snippet.strip())
                snippet = ""
    if obr != 0:
        rospy.logwarn("incorrect PDDL (not matching brackets) passed to tokenizePDDL(): %s", pddlString)
    return tokens

def mergeStatePDDL(PDDLone, PDDLtwo):
    '''
    This function merges PDDLone into PDDLtwo (in place)
    '''
    #rospy.logwarn("PDDL1: %s, PDDL2: %s",str(PDDLone), str(PDDLtwo))
    
    for x in tokenizePDDL(PDDLone.statement):
        # find out sensorName and type of currently processed token (must be declared in either functions or predicates sets)
        sensorName = ""
        statementIsPredicate = False
        for sn in PDDLone.predicates:
            if sn in x:
                sensorName = sn
                statementIsPredicate = True
                break
        if not sensorName:
            for sn in PDDLone.functions:
                if sn in x:
                    sensorName = sn
                    break
        if not sensorName: # sanity check
            rospy.logwarn("inconsistent PDDL data structure: statement %s does not contain any of the declared in predicates (%s) or functions (%s)", x, PDDLone.predicates, PDDLone.functions)
        if statementIsPredicate:
            if sensorName in PDDLtwo.predicates:
                if not x in tokenizePDDL(PDDLtwo.statement):
                    rospy.logwarn("predicate declared differently than before %s vs %s", x, PDDLtwo.statement)
            else:
                PDDLtwo.predicates.add(sensorName)
                PDDLtwo.statement += "\n\t\t{0}".format(x)
        else: # statement is function
            if sensorName in PDDLtwo.functions:
                if not x in tokenizePDDL(PDDLtwo.statement):
                    rospy.logwarn("function declared differently than before %s vs %s", x, PDDLtwo.statement)
            else:
                PDDLtwo.functions.add(sensorName)
                PDDLtwo.statement += "\n\t\t{0}".format(x)
    return PDDLtwo

def parseStatePDDL(pddl):
    '''
    This function creates a {sensor name <string> : value} dictionary of a state PDDL object.
    '''
    state = {}
    for token in tokenizePDDL(pddl.statement):
        match =  functionRegex.search(token)
        if match: # it is a function value declaration
            state[match.group(1)] = float(match.group(2))
        else: # it must be a predicate
            match =  predicateRegex.search(token)
            if match: # it actually is a predicate
                if match.group(2): # it is negated
                    state[match.group(3)] = False
                else:
                    state[match.group(3)] = True
    return state
            
        
def getStatePDDLchanges(oldPDDL, newPDDL):
    '''
    This function creates a {sensor name <string> : indicator <float [-1 - 1]} dictionary of state changes.
    '''
    changes = {}
    oldState = parseStatePDDL(oldPDDL)
    newState = parseStatePDDL(newPDDL)
    for (sensorName, value) in newState.iteritems():
        if sensorName in oldState:
            if isinstance(value, bool): # it is a predicate
                if oldState[sensorName] == True and value == False:
                    changes[sensorName] = -1.0
                if oldState[sensorName] == False and value == True:
                    changes[sensorName] = 1.0
            else: # it must be float then
                if value != oldState[sensorName]:
                    changes[sensorName] = value - oldState[sensorName]
    return changes
                