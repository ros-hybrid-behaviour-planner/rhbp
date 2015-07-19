'''
Created on 22.04.2015

@author: stephan
'''

import operator
import conditions
import warnings
import itertools
import rospy
from behaviourPlannerPython.srv import AddGoal, GetStatus, GetStatusResponse
from behaviourPlannerPython.msg import Wish

class Goal(object): #TODO: Although it is similar to Preconditions of a behaviour we accept the code duplication for now
    '''
    This class represents a goal. Goals have conditions that need to be fulfilled.
    '''
    
    def __init__(self, name, permanent):
        '''
        Constructor
        '''
        self._name = name
        self._wishes = {}       # Stores wishes exactly like correlations. We get this via getStatus service of actual goal node
        self._fulfillment = 0.0 # We get it via getStatus service of actual goal node
        self._isPermanent = permanent

    
    def fetchStatus(self):
        '''
        This method fetches the status from the actual behaviour node via GetStatus service call
        '''
        rospy.logdebug("Waiting for service %s", self._name + 'GetStatus')
        rospy.wait_for_service(self._name + 'GetStatus')
        try:
            getStatusRequest = rospy.ServiceProxy(self._name + 'GetStatus', GetStatus)
            status = getStatusRequest()
            self._fulfillment = status.satisfaction
            self._wishes = dict([(wish.sensorName, wish.indicator) for wish in status.wishes])
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
    
    def __str__(self):
        return self._name
    
    def __repr__(self):
        return self._name

class GoalBase(object):
    '''
    This is the base class for goals in python
    '''

    def __init__(self, name, permanent = True, conditions = []):
        '''
        Constructor
        '''
        self._name = name # a unique name is mandatory
        self._isPermanent = permanent
        self._getActivationService = rospy.Service(self._name + 'GetStatus', GetStatus, self.getStatus)
        self._conditions = conditions
        rospy.loginfo("GoalBase constructor waiting for registration at planner manager for behaviour node %s", self._name)
        rospy.wait_for_service('AddBehaviour')
        try:
            registerMe = rospy.ServiceProxy('AddGoal', AddGoal)
            registerMe(self._name, self._isPermanent)
            rospy.loginfo("GoalBase constructor registered at planner manager for goal node %s", self._name)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in GoalBase constructor (for goal node %s): %s", self._name, e)
    
    def __del__(self):
        '''
        Destructor
        '''
        try:
            self._getStatusService.shutdown()
        except Exception as e:
            rospy.logerr("Fucked up in destructor of GoalBase: %s", e)
    
    def computeSatisfaction(self):
        """
        This method should return the satisfaction of the conditions (the readiness) as float [0 to 1].
        """
        return reduce(operator.mul, (x.satisfaction for x in self._conditions), 1)
    
    def computeWishes(self):
        """
        This method should return a list of Wish messages indicating the desired sensor changes that would satisfy its conditions.
        A Wish message is constructed from a string (sensor name) and a desire indicator (float, [-1 to 1]).
        """
        return [Wish(item[0].name, item[1]) for item in list(itertools.chain.from_iterable([x.getWishes() for x in self._conditions]))]
    
    def getStatus(self, request):
        return GetStatusResponse(**{
                                  "satisfaction" : self.computeSatisfaction(),
                                  "wishes"       : self.computeWishes()
                                })
    
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