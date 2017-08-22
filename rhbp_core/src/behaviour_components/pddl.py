'''
Created on 06.10.2015

@author: wypler, hrabia
'''
from __future__ import division # force floating point division when using plain /
import re
import rospy

functionRegex = re.compile(r'\s*\(\s*=\s*\(\s*([a-zA-Z0-9_\.-]+)\s*\)\s*([-+]?[0-9]*\.?[0-9]+)\s*\)\s*')
predicateRegex = re.compile(r'\s*\(\s*((not)\s*\()?\s*([a-zA-Z0-9_\.-]+)\s*\)?\s*\)\s*')

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.planning')

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

    @property
    def empty(self):
        return not self.statement.strip() and len(self.predicates) == 0 and len(self.predicates) == 0
    
    def __repr__(self):
        return "PDDL: predicates: " + " ".join(self.predicates) + " functions: " + " ".join(self.functions) + " statement: " + self.statement


def get_pddl_effect_name(sensor_name, activator_name):
    """
    generate an effect name that uniquely identifies a particular effect on a activator/sensor combination
    :param sensor_name: name of the sensor
    :param activator_name: name of the activator
    :return: str pddl effect name
    """
    if activator_name:
        return sensor_name  + '_' + activator_name
    else:
        return sensor_name

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


def init_missing_functions(domain_pddl, state_pddl):
    '''
    This function initialises all functions in the state pddl which are mentioned in the domain pddl but not yet initialised
    :param domain_pddl: the domain pddl object
    :param state_pddl: the state pddl object
    :return: adjusted state_pddl
    '''
    uninitialized_functions = domain_pddl.functions.difference(state_pddl.functions)

    state_pddl.statement += "".join("\n\t\t( = ({0}) 0)".format(x) for x in uninitialized_functions)

    state_pddl.functions = state_pddl.functions.union(uninitialized_functions)

    return state_pddl

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

def create_valid_pddl_name(str):
    """
    Function converts a given string into a pddl valid name
    :param str: string to convert
    :type str: String
    """
    return ''.join(e for e in str if e.isalnum() or e =='_')

