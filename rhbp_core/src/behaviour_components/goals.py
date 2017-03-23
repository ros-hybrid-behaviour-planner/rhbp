'''
Created on 22.04.2015

@author: wypler, hrabia, rieger
'''

import itertools
import operator
import warnings

import rospy
from std_msgs.msg import Bool

from rhbp_core.msg import Wish, Status
from rhbp_core.srv import AddGoal, GetStatus, GetStatusResponse, Activate, ActivateResponse, GetPDDL, GetPDDLResponse, \
    SetInteger, SetIntegerResponse
from .activators import Condition, BooleanActivator
from .conditions import Conditonal
from .pddl import PDDL, mergeStatePDDL
from .sensors import SimpleTopicSensor
from utils.misc import FinalInitCaller


class AbstractGoalRepresentation(object):
    '''
    This class represents a goal in the manager.
    '''

    def __init__(self, name, permanent=False, satisfaction_threshold=1.0, priority=0, active=True, activated=True):
        self._name = name
        self._isPermanent = permanent
        self._priority = priority
        self._satisfaction_threshold = satisfaction_threshold
        self.__wishes = []
        self._fulfillment = 0.0
        self._active = active  # This indicates (if True) that there have been no severe issues in the actual goal node
        # and the goal can be expected to be operational. If the actual goal reports ready == False
        # we will ignore it in activation computation.
        self._activated = activated  # This member only exists as proxy for the corrsponding actual goal's property.
        # It is here because of the comprehensive status message,
        # published each step by the manager for rqt

    @property
    def name(self):
        return self._name

    @property
    def isPermanent(self):
        return self._isPermanent

    @property
    def priority(self):
        return self._priority

    @priority.setter
    def priority(self, value):
        self._priority = value

    def __str__(self):
        return self._name

    def __repr__(self):
        return self._name

    @property
    def wishes(self):
        return self.__wishes

    @wishes.setter
    def wishes(self, wishes):
        '''

        :param wishes: list of (sensor name <string> : indicator <float> [-1 to 1])
        '''
        self.__wishes = wishes

    @property
    def active(self):
        return self._active

    @active.setter
    def active(self, value):
        self._active = value

    @property
    def activated(self):
        return self._activated

    @activated.setter
    def activated(self, value):
        self._activated = value

    @property
    def fulfillment(self):
        return self._fulfillment

    @fulfillment.setter
    def fulfillment(self, value):
        self._fulfillment = value

    @property
    def satisfaction_threshold(self):
        return self._satisfaction_threshold

    @satisfaction_threshold.setter
    def satisfaction_threshold(self, value):
        self._satisfaction_threshold = value

    @property
    def satisfied(self):
        return self._fulfillment >= self._satisfaction_threshold

    def fetchPDDL(self):
        '''
        This method generates pddl.
        It returns a tuple of (goal_pddl, state_pddl).
        '''
        raise NotImplementedError()

    def sync(self):
        '''
        Update satisfaction, wishes and all other stuff
        '''
        raise NotImplementedError()


class Goal(object):
    '''
    This class is the base class of goals. All calculation logic is placed here.
    Therefore conditions can registered, which are used for satisfaction computation
    '''

    def __init__(self, name, conditions=[], satisfaction_threshold=1.0, priority=0, active=True, activated=True):
        '''
        Constructor
        '''
        self._activateService = rospy.Service(name + 'Activate', Activate, self.activateCallback)
        self._priorityService = rospy.Service(name + 'Priority', SetInteger, self.setPriorityCallback)
        self._conditions = []
        self._conditions.extend(conditions)
        self._satisfaction_threshold = satisfaction_threshold  # treshhold that defines when the goal is satisfied/fulfilled from the preconditions
        self._active = active  # if anything in the goal is not initialized or working properly this must be set to False and communicated via getStatus service
        self._activated = activated  # The activate Service sets the value of this property.
        self._priority = priority  # The higher the (unsigned) number the higher the importance

    def __del__(self):
        '''
        Destructor
        '''
        try:
            self._activateService.shutdown()
            self._priorityService.shutdown();
        except Exception as e:
            rospy.logerr("Fucked up in destructor of GoalBase: %s", e)

    def updateComputation(self):
        """
        Updates all subentities of the behaviour in order to do computations only once
        """
        for p in self._conditions:
            p.sync()
        for p in self._conditions:
            p.updateComputation()

    def computeSatisfaction(self):
        """
        This method should return the satisfaction of the conditions (the readiness) as float [0 to 1].
        """
        try:
            return reduce(operator.mul, (x.satisfaction for x in self._conditions), 1)
        except AssertionError:  # this probably comes from an uninitialized sensor or not matching activator for the sensor's data type in at least one precondition
            self._active = False
            return 0.0

    def computeWishes(self):
        """
        This method should return a list of Wish messages indicating the desired sensor changes that would satisfy its conditions.
        A Wish message is constructed from a string (effect name) and a desire indicator (float, [-1 to 1]).
        """
        try:
            return [Wish(item[0], item[1]) for item in
                    list(itertools.chain.from_iterable([x.getWishes() for x in self._conditions]))]
        except AssertionError:  # this probably comes from an uninitialized sensor or not matching activator for the sensor's data type in at least one precondition
            self._active = False
            return []

    def getGoalStatements(self):
        return " ".join([x.getPreconditionPDDL(self._satisfaction_threshold).statement for x in self._conditions])

    def getStatePDDL(self):
        pddl = PDDL()
        for c in self._conditions:
            for s in c.getStatePDDL():  # it is a list
                pddl = mergeStatePDDL(s, pddl)
        return pddl

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
        if issubclass(type(condition), Conditonal):
            self._conditions.append(condition)
        else:
            warnings.warn("That's no conditional object!")

    @property
    def activated(self):
        return self._activated

    @property
    def priority(self):
        return self._priority

    @property
    def satisfaction_threshold(self):
        return self._satisfaction_threshold

    @property
    def active(self):
        return self._active


class GoalProxy(AbstractGoalRepresentation):
    '''
    This class connects a remote goal with the manager.
    Is instanciated automatically. Don't instanciate manually
    '''

    def __init__(self, name, permanent):
        '''
        Constructor
        '''
        super(GoalProxy, self).__init__(name, permanent)

    def fetchPDDL(self):
        '''
        This method fetches the PDDL from the actual goal node via GetPDDLservice call
        '''
        rospy.logdebug("Waiting for service %s", self._name + 'PDDL')
        rospy.wait_for_service(self._name + 'PDDL')
        try:
            getPDDLRequest = rospy.ServiceProxy(self._name + 'PDDL', GetPDDL)
            pddl = getPDDLRequest()
            return (PDDL(statement=pddl.goalStatement),
                    PDDL(statement=pddl.stateStatement, predicates=pddl.statePredicates,
                         functions=pddl.stateFunctions))
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in fetchPDDL of %s: %s", self._name, e)

    def sync(self):
        '''
        This method fetches the status from the actual goal node via GetStatus service call
        '''
        rospy.logdebug("Waiting for service %s", self._name + 'GetStatus')
        rospy.wait_for_service(self._name + 'GetStatus')
        try:
            get_status_request = rospy.ServiceProxy(self._name + 'GetStatus', GetStatus)
            status = get_status_request().status
            self.fulfillment = status.satisfaction
            self.wishes = [(wish.sensorName, wish.indicator) for wish in status.wishes]
            self.active = status.active
            self.activated = status.activated
            self.priority = status.priority
            self.satisfaction_threshold = status.threshold
            if self._name != status.name:
                rospy.logerr("%s fetched a status message from a different goal: %s. This cannot happen!",
                             self._name,
                             status.name)
            rospy.logdebug("%s reports the following status:\nfulfillment %s\nwishes %s", self.name,
                           self.fulfillment,
                           self.wishes)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in GetStatus of %s: %s", self._name, e)


# TODO Rename to RemoteGoal
class GoalBase(Goal):
    '''
    Goal, which is automatically registered at the manager. Therefore the ros messages are used.
    Allows to be on different node than the manager.
    '''

    __metaclass__ = FinalInitCaller

    def __init__(self, name, permanent=False, conditions=[], plannerPrefix="", priority=0, satisfaction_threshold=1.0):
        '''

        :param name:  a unique name is mandatory
        :param permanent:
        :param conditions:
        :param plannerPrefix: if you have multiple planners in the same ROS environment use a prefix to identify the right one.
        :param priority:
        :param satisfaction_threshold:
        '''
        super(GoalBase, self).__init__(name=name, conditions=conditions, satisfaction_threshold=satisfaction_threshold,
                                       priority=priority)
        self._name = name

        self._getStatusService = rospy.Service(self._name + 'GetStatus', GetStatus, self.getStatus)
        self._pddlService = rospy.Service(self._name + 'PDDL', GetPDDL, self.pddlCallback)
        self._permanent = permanent
        self._planner_prefix = plannerPrefix

    def final_init(self):
        """
        Ensure registration after the entire initialisation (including sub classes) is done
        """
        self._register_goal()

    def _register_goal(self):
        try:
            rospy.logdebug(
                "GoalBase constructor waiting for registration at planner manager with prefix '%s' for behaviour node %s",
                self._planner_prefix, self._name)
            rospy.wait_for_service(self._planner_prefix + 'AddGoal')
            registerMe = rospy.ServiceProxy(self._planner_prefix + 'AddGoal', AddGoal)
            registerMe(self._name, self._permanent)
            rospy.logdebug("GoalBase constructor registered at planner manager with prefix '%s' for goal node %s",
                           self._planner_prefix, self._name)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in GoalBase constructor (for goal node %s): %s", self._name, e)

    def __del__(self):
        '''
        Destructor
        '''
        try:
            self._getStatusService.shutdown()
            self._pddlService.shutdown()
            super(GoalBase, self).__del__()
        except Exception as e:
            rospy.logerr("Error in destructor of GoalBase: %s", e)

    def pddlCallback(self, dummy):
        self.updateComputation()
        goalStatements = self.getGoalStatements()
        statePDDL = self.getStatePDDL()
        return GetPDDLResponse(**{"goalStatement": goalStatements,
                                  "stateStatement": statePDDL.statement,
                                  "statePredicates": list(statePDDL.predicates),
                                  "stateFunctions": list(statePDDL.functions)})

    def getStatus(self, request):
        self.updateComputation()
        self._active = self._activated
        status = Status(**{
            "name": self._name,
            "satisfaction": self.computeSatisfaction(),
            "wishes": self.computeWishes(),
            "active": self._active,
            "activated": self._activated,
            "priority": self._priority,
            "threshold": self._satisfaction_threshold
        })
        return GetStatusResponse(status)


class OfflineGoal(AbstractGoalRepresentation):
    '''
    This class represents a goal, which is registered directly at the manager.
    As consequence no messages are used for communicating with the manager.
    '''

    def __init__(self, name, permanent=False, conditions=[], priority=0, satisfaction_threshold=1.0):
        '''
        Constructor
        '''
        super(OfflineGoal, self).__init__(name=name, permanent=permanent,
                                          satisfaction_threshold=satisfaction_threshold, priority=priority)
        self.__goal = Goal(name=name, satisfaction_threshold=satisfaction_threshold)
        for condition in conditions:
            self.add_condition(condition)

    def sync(self):
        self.__goal.updateComputation()
        self.fulfillment = self.__goal.computeSatisfaction()
        self.wishes = [(wish.sensorName, wish.indicator) for wish in self.__goal.computeWishes()]
        self._priority = self.__goal.priority
        self.active = self.__goal.activated

    def fetchPDDL(self):
        self.__goal.updateComputation()
        goal_statements = self.__goal.getGoalStatements()
        state_pddl = self.__goal.getStatePDDL()
        return (PDDL(statement=goal_statements),
                PDDL(statement=state_pddl.statement, predicates=list(state_pddl.predicates), functions=list(
                    state_pddl.functions)))

    def add_condition(self, condition):
        self.__goal.addCondition(condition)


class PublisherGoal(GoalBase):
    """
    Goal class which publishes its activation state as ROS topic
    """

    def __init__(self, name, permanent=False, conditions=[], plannerPrefix="", priority=0, satisfaction_threshold=1.0):
        """
        Without manual goal activation/deaction only permanent=False does make sense
        """
        super(PublisherGoal, self).__init__(name=name, permanent=permanent, conditions=conditions,
                                            plannerPrefix=plannerPrefix, priority=priority,
                                            satisfaction_threshold=satisfaction_threshold)

        self.__topic_name = self._name + "_activated"
        self.__pub = rospy.Publisher(self.__topic_name, Bool, queue_size=10)

    def updateComputation(self):
        super(PublisherGoal, self).updateComputation()
        self.__pub.publish(self._activated)

    def create_condition(self):
        """
        Creating a new condition object based on the positive goal activation state --> full activation on activated goal
        :returns new condition object
        """
        condition_name = self._name + "_condition"
        sensor_name = self._name + "_sensor"
        sensor = SimpleTopicSensor(name=sensor_name, topic=self.__topic_name, message_type=Bool,
                                   initial_value=self._activated)
        activator = BooleanActivator()
        return Condition(name=condition_name, sensor=sensor, activator=activator)
