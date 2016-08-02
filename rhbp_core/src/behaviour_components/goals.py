'''
Created on 22.04.2015

@author: stephan
'''

import operator
import conditions
import warnings
import itertools
import rospy
from rhbp_core.msg import Wish, Status
from rhbp_core.srv import AddGoal, GetStatus, GetStatusResponse, Activate, ActivateResponse, GetPDDL, GetPDDLResponse, SetInteger, SetIntegerResponse
from util import PDDL, mergeStatePDDL


class Goal(object):
    '''
    This class represents a goal. Goals have conditions that need to be fulfilled.
    '''
    
    def __init__(self, name, permanent):
        '''
        Constructor
        '''
        self._name = name
        self._wishes = []       # Stores wishes s list of (sensor name <string> : indicator <float> [-1 to 1]) tuples. We get this via getStatus service of actual goal node
        self._fulfillment = 0.0 # We get it via getStatus service of actual goal node
        self._active = True     # This indicates (if True) that there have been no severe issues in the actual goal node and the goal can be expected to be operational. If the actual goal reports ready == False we will ignore it in activation computation.
        self._activated = True  # This member only exists as proxy for the corrsponding actual goal's property. It is here because of the comprehensive status message published each step by the manager for rqt
        self._isPermanent = permanent
        self._priority = 0

    def fetchPDDL(self):
        '''
        This method fetches the PDDL from the actual behaviour node via GetPDDLservice call
        It returns a tuple of (goal_pddl, state_pddl).
        '''
        rospy.logdebug("Waiting for service %s", self._name + 'PDDL')
        rospy.wait_for_service(self._name + 'PDDL')
        try:
            getPDDLRequest = rospy.ServiceProxy(self._name + 'PDDL', GetPDDL)
            pddl = getPDDLRequest()
            return (PDDL(statement = pddl.goalStatement), PDDL(statement = pddl.stateStatement, predicates = pddl.statePredicates, functions = pddl.stateFunctions))
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in fetchPDDL of %s: %s", self._name, e)
    
    def fetchStatus(self):
        '''
        This method fetches the status from the actual behaviour node via GetStatus service call
        '''
        rospy.logdebug("Waiting for service %s", self._name + 'GetStatus')
        rospy.wait_for_service(self._name + 'GetStatus')
        try:
            getStatusRequest = rospy.ServiceProxy(self._name + 'GetStatus', GetStatus)
            status = getStatusRequest().status
            self._fulfillment = status.satisfaction
            self._wishes = [(wish.sensorName, wish.indicator) for wish in status.wishes]
            self._active = status.active
            self._activated = status.activated
            self._priority = status.priority
            if self._name != status.name:
                rospy.logerr("%s fetched a status message from a different goal: %s. This cannot happen!", self._name, status.name)
            rospy.logdebug("%s reports the following status:\nfulfillment %s\nwishes %s", self._name, self._fulfillment, self._wishes)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in GetStatus of %s: %s", self._name, e)
    
    @property
    def name(self):
        return self._name
    
    @property
    def wishes(self):
        return self._wishes
    
    @property
    def fulfillment(self):
        return self._fulfillment
    
    @property
    def isPermanent(self):
        return self._isPermanent
    
    @property
    def active(self):
        return self._active
    
    @active.setter
    def active(self, value):
        self._active = value
    
    @property
    def activated(self):
        return self._activated
    
    @property
    def priority(self):
        return self._priority
    
    def __str__(self):
        return self._name
    
    def __repr__(self):
        return self._name
    

class GoalBase(object):
    '''
    This is the base class for goals in python
    '''
    def __init__(self, name, permanent = False, conditions = [], plannerPrefix = "", priority = 0):
        '''
        Constructor
        '''
        self._name = name # a unique name is mandatory
        self._isPermanent = permanent
        self._getStatusService = rospy.Service(self._name + 'GetStatus', GetStatus, self.getStatus)
        self._activateService = rospy.Service(self._name + 'Activate', Activate, self.activateCallback)
        self._pddlService = rospy.Service(self._name + 'PDDL', GetPDDL, self.pddlCallback)
        self._priorityService = rospy.Service(self._name + 'Priority', SetInteger, self.setPriorityCallback)
        self._conditions = conditions
        self._plannerPrefix = plannerPrefix # if you have multiple planners in the same ROS environment use a prefix to identify the right one.
        self._active = True # if anything in the goal is not initialized or working properly this must be set to False and communicated via getStatus service
        self._activated = True # The activate Service sets the value of this property.
        self._priority = priority # The higher the (unsigned) number the higher the importance

        try:
            rospy.logdebug("GoalBase constructor waiting for registration at planner manager with prefix '%s' for behaviour node %s", self._plannerPrefix, self._name)
            rospy.wait_for_service(self._plannerPrefix + 'AddGoal')
            registerMe = rospy.ServiceProxy(self._plannerPrefix + 'AddGoal', AddGoal)
            registerMe(self._name, self._isPermanent)
            rospy.logdebug("GoalBase constructor registered at planner manager with prefix '%s' for goal node %s", self._plannerPrefix, self._name)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in GoalBase constructor (for goal node %s): %s", self._name, e)
    
    def __del__(self):
        '''
        Destructor
        '''
        try:
            self._getStatusService.shutdown()
            self._activateService.shutdown()
            self._pddlService.shutdown();
            self._priorityService.shutdown();
        except Exception as e:
            rospy.logerr("Fucked up in destructor of GoalBase: %s", e)
            
    def updateComputation(self):
        """
        Updates all subentities of the behaviour in order to do computations only once
        """
        for p in self._conditions:
            p.updateComputation()
    
    def computeSatisfaction(self):
        """
        This method should return the satisfaction of the conditions (the readiness) as float [0 to 1].
        """
        try:
            return reduce(operator.mul, (x.satisfaction for x in self._conditions), 1)
        except AssertionError: # this probably comes from an uninitialized sensor or not matching activator for the sensor's data type in at least one precondition
            self._active = False
            return 0.0
    
    def computeWishes(self):
        """
        This method should return a list of Wish messages indicating the desired sensor changes that would satisfy its conditions.
        A Wish message is constructed from a string (sensor name) and a desire indicator (float, [-1 to 1]).
        """
        try:
            return [Wish(item[0].name, item[1]) for item in list(itertools.chain.from_iterable([x.getWishes() for x in self._conditions]))]
        except AssertionError: # this probably comes from an uninitialized sensor or not matching activator for the sensor's data type in at least one precondition
            self._active = False
            return []

    def getGoalStatements(self):
        return " ".join([x.getPreconditionPDDL().statement for x in self._conditions])
    
    def getStatePDDL(self):
        pddl = PDDL()
        for c in self._conditions :
            for s in c.getStatePDDL(): # it is a list
                pddl = mergeStatePDDL(s, pddl)
        return pddl                      

    def pddlCallback(self, dummy):
        self.updateComputation()
        goalStatements = self.getGoalStatements()
        statePDDL = self.getStatePDDL()
        return GetPDDLResponse(**{"goalStatement" : goalStatements,
                                  "stateStatement" : statePDDL.statement,
                                  "statePredicates" : list(statePDDL.predicates),
                                  "stateFunctions" : list(statePDDL.functions)})
    
    def getStatus(self, request):
        self.updateComputation()
        self._active = self._activated
        status = Status(**{
                        "name"         : self._name,
                        "satisfaction" : self.computeSatisfaction(),
                        "wishes"       : self.computeWishes(),
                        "active"       : self._active,
                        "activated"    : self._activated,
                        "priority"     : self._priority
                       })
        return GetStatusResponse(status)
    
    def activateCallback(self, request):
        '''
        This method activates or deactivates the goal.
        This method must not block.
        '''
        self._activated = request.active
        return ActivateResponse()
    
    def setPriorityCallback(self, request):
        self._priority = request.value
        return SetIntegerResponse()
    
    def addCondition(self, condition):
        '''
        This method adds a condition to the goal.
        It is not mandatory to use this method at all but it may make development easier because the default implementations of computeActivation(), computeSatisfaction(), and computeWishes work with the preconditions added here.
        If you don't want to use this mechanism then you HAVE TO implement those yourself!
        There is an AND relationship between all elements (all have to be fulfilled so that the behaviour is ready)
        To enable OR semantics use the Disjunction object.
        '''
        if issubclass(type(condition), conditions.Conditonal):
            self._conditions.append(condition)
        else:
            warnings.warn("That's no conditional object!")
