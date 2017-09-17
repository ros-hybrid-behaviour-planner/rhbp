'''
Created on 22.04.2015

@author: wypler, hrabia, rieger
'''

import itertools
import operator
import warnings
import traceback

from abc import ABCMeta, abstractmethod

import rospy
from std_msgs.msg import Bool

from rhbp_core.msg import Status
from rhbp_core.srv import AddGoal, GetStatus, GetStatusResponse, Activate, ActivateResponse, GetPDDL, GetPDDLResponse, \
    SetInteger, SetIntegerResponse, RemoveGoal
from .activators import Condition, BooleanActivator
from .conditions import Conditonal
from .condition_elements import Wish
from .pddl import PDDL, mergeStatePDDL
from .sensors import SimpleTopicSensor
from utils.misc import FinalInitCaller

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.goals')


class AbstractGoalRepresentation(object):
    '''
    This class represents a goal in the manager.
    '''

    __metaclass__ = ABCMeta

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
        """
        :return: list of (Wish())
        """
        return self.__wishes

    @wishes.setter
    def wishes(self, wishes):
        '''
        :param wishes: list of (Wish())
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

    @abstractmethod
    def fetchPDDL(self):
        '''
        This method generates pddl.
        It returns a tuple of (goal_pddl, state_pddl).
        '''
        raise NotImplementedError()

    @abstractmethod
    def fetchStatus(self, current_step):
        """
        Update satisfaction, wishes and all other stuff
        :param current_step: 
        """
        raise NotImplementedError()


class Goal(object):
    '''
    This class is the base class of goals. All calculation logic is placed here.
    Therefore conditions can registered, which are used for satisfaction computation
    '''

    def __init__(self, name, planner_prefix, conditions=None, satisfaction_threshold=1.0, priority=0, active=True, activated=True):
        '''
        Constructor
        '''
        self._name = name
        self._planner_prefix = planner_prefix
        if conditions is None:
            self._conditions = []
        else:
            self._conditions = conditions
        self._satisfaction_threshold = satisfaction_threshold  # treshhold that defines when the goal is satisfied/fulfilled from the preconditions
        self._active = active  # if anything in the goal is not initialized or working properly this must be set to False and communicated via getStatus service
        self._activated = activated  # The activate Service sets the value of this property.
        self._priority = priority  # The higher the (unsigned) number the higher the importance
        self._services_running = False
        self._init_services()

    def _init_services(self):
        service_prefix = self._planner_prefix + '/' + self._name + '/'

        #these are convenience services which can be called remotely in order to configure currently active goals and priorities
        self._activateService = rospy.Service(service_prefix + 'Activate', Activate, self._activateCallback)
        self._priorityService = rospy.Service(service_prefix + 'Priority', SetInteger, self.setPriorityCallback)
        self._services_running = True

    def _cleanup_topics_services(self):
        """
        Cleaning up ROS communication interface
        """
        if self._services_running:
            self._activateService.shutdown()
            self._priorityService.shutdown()
            self._services_running = False

    def __del__(self):
        '''
        Destructor
        '''
        try:
            self._cleanup_topics_services()
        except Exception:
            rhbplog.logerr("Destructor of GoalBase failed: %s", traceback.format_exc())

    def updateComputation(self, manager_step):
        """
        Updates all subentities of the behaviour in order to do computations only once
        :param manager_step: current planning decision_making step of the manager
        """
        synchronized_conditions = []
        for p in self._conditions:
            if p.need_update(manager_step):
                p.sync()
                synchronized_conditions.append(p)
        for p in synchronized_conditions:
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
            return [item.get_wish_msg() for item in
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

    def _activateCallback(self, request):
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

    @activated.setter
    def activated(self, activated):
        self._activated = activated

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
    It is instantiated automatically. Don't instantiated it manually
    '''

    SERVICE_TIMEOUT = 2

    def __init__(self, name, permanent, planner_prefix):
        '''
        Constructor
        '''
        super(GoalProxy, self).__init__(name, permanent)
        self._service_prefix = planner_prefix + '/' + self._name + '/'

    def fetchPDDL(self):
        '''
        This method fetches the PDDL from the actual goal node via GetPDDLservice call
        '''
        try:
            rhbplog.logdebug("Waiting for service %s", self._service_prefix + 'PDDL')
            rospy.wait_for_service(self._service_prefix + 'PDDL')
        except rospy.ROSException:
            self._handle_service_timeout()
            return
        try:
            getPDDLRequest = rospy.ServiceProxy(self._service_prefix + 'PDDL', GetPDDL)
            pddl = getPDDLRequest()
            return (PDDL(statement=pddl.goalStatement),
                    PDDL(statement=pddl.stateStatement, predicates=pddl.statePredicates,
                         functions=pddl.stateFunctions))
        except rospy.ServiceException:
            rhbplog.logerr("ROS service exception in 'fetchPDDL' of goal '%s': %s", self._name,
                         traceback.format_exc())

    def fetchStatus(self, current_step):
        """
        This method fetches the status from the actual goal node via GetStatus service call
        :param current_step: 
        """

        try:
            rhbplog.logdebug("Waiting for service %s", self._service_prefix + 'GetStatus')
            rospy.wait_for_service(self._service_prefix + 'GetStatus', timeout=self.SERVICE_TIMEOUT)
        except rospy.ROSException:
            self._handle_service_timeout()
            return

        try:
            get_status_request = rospy.ServiceProxy(self._service_prefix + 'GetStatus', GetStatus)
            status = get_status_request(current_step).status
            self.fulfillment = status.satisfaction
            self.wishes = [Wish.from_wish_msg(wish) for wish in status.wishes]
            self.active = status.active
            self.activated = status.activated
            self.priority = status.priority
            self.satisfaction_threshold = status.threshold
            if self._name != status.name:
                rhbplog.logerr("%s fetched a status message from a different goal: %s. This cannot happen!",
                             self._name,
                             status.name)
            rhbplog.logdebug("%s reports the following status:\nfulfillment %s\nwishes %s", self.name,
                           self.fulfillment,
                           self.wishes)
        except rospy.ServiceException:
            rhbplog.logerr("ROS service exception in 'fetchStatus' of goal '%s': %s", self._name,
                         traceback.format_exc())

    def _handle_service_timeout(self):
        """
        basically deactivate the goal in case a service has timeout
        """
        rhbplog.logerr("ROS service timeout of goal '%s': %s. Fulfillment will be reset", self._name,
                     traceback.format_exc())
        self._active = False
        self.fulfillment = 0.0

    @AbstractGoalRepresentation.activated.setter
    def activated(self, value):
        self._activated = value
        try:
            #inform remote goal about new activated state
            service_name = self._service_prefix + 'Activate'
            rhbplog.logdebug("Waiting for service %s", service_name)
            rospy.wait_for_service(service_name)
            activateRequest = rospy.ServiceProxy(service_name, Activate)
            activateRequest(value)
        except rospy.ServiceException:
            rhbplog.logerr("ROS service exception in 'activated' of goal '%s': %s", self._name,
                         traceback.format_exc())


# TODO Rename to RemoteGoal
class GoalBase(Goal):
    '''
    Goal, which is automatically registered at the manager. Therefore the ros messages are used.
    Allows to be on different node than the manager.
    '''

    __metaclass__ = FinalInitCaller

    SERVICE_TIMEOUT = 5

    def __init__(self, name, permanent=False, conditions=None, plannerPrefix="", priority=0, satisfaction_threshold=1.0):
        '''

        :param name:  a unique name is mandatory
        :param permanent:
        :param conditions:
        :param plannerPrefix: if you have multiple planners in the same ROS environment use a prefix to identify the right one.
        :param priority:
        :param satisfaction_threshold:
        '''
        super(GoalBase, self).__init__(name=name, planner_prefix=plannerPrefix, conditions=conditions, satisfaction_threshold=satisfaction_threshold,
                                       priority=priority)
        self._name = name

        self._permanent = permanent
        self._planner_prefix = plannerPrefix
        self._registered = False  # keeps track of goal registration state


    def _init_services(self):
        super(GoalBase, self)._init_services()
        self._service_prefix = self._planner_prefix + '/' + self._name + '/'
        self._getStatusService = rospy.Service(self._service_prefix + 'GetStatus', GetStatus, self.getStatus)
        self._pddlService = rospy.Service(self._service_prefix + 'PDDL', GetPDDL, self.pddlCallback)

    def final_init(self):
        """
        Ensure registration after the entire initialisation (including sub classes) is done
        """
        self.register()

    def register(self):
        """
        register goal at the manager, this is automatically called during initialization/construction
        only call it manually if you have manually unregistred the goal before
        """
        if self._registered:
            rhbplog.logwarn("Goal '%s' is already registred", self._name)
            return
        try:
            rhbplog.logdebug(
                "GoalBase constructor waiting for registration at planner manager with prefix '%s' for behaviour node %s",
                self._planner_prefix, self._name)
            rospy.wait_for_service(self._planner_prefix + '/' + 'AddGoal')
            registerMe = rospy.ServiceProxy(self._planner_prefix + '/' + 'AddGoal', AddGoal)
            registerMe(self._name, self._permanent)
            self._registered = True
            rhbplog.logdebug("GoalBase constructor registered at planner manager with prefix '%s' for goal node %s",
                           self._planner_prefix, self._name)
        except rospy.ServiceException:
            rhbplog.logerr("ROS service exception in '_register_goal' of goal '%s': %s", self._name,
                         traceback.format_exc())

    def _cleanup_topics_services(self):
        """
        Cleaning up ROS communication interface
        """
        super(GoalBase, self)._cleanup_topics_services()
        self._getStatusService.shutdown()
        self._pddlService.shutdown()

    def unregister(self, terminate_services=True):
        """
        Remove/Unregister behaviour from the manager
        :param terminate_services: True for shuting down all service interfaces as well
        """
        try:
            service_name = self._planner_prefix + '/' + 'RemoveGoal'
            rhbplog.logdebug("Waiting for service %s", service_name)
            # do not wait forever here, manager might be already closed
            rospy.wait_for_service(service_name, timeout=self.SERVICE_TIMEOUT)
            try:
                remove_goal = rospy.ServiceProxy(service_name, RemoveGoal)
                remove_goal(self._name)
            except rospy.ServiceException:
                rhbplog.logerr("ROS service exception in 'unregister' of goal '%s': %s", self._name,
                               traceback.format_exc())
        except rospy.ROSException:
            # if the service is not available this is not crucial.
            pass

        self._registered = False
        if terminate_services:
            self._cleanup_topics_services()

    def __del__(self):
        '''
        Destructor
        '''
        try:
            if self._registered:
                self.unregister(terminate_services=True)

            super(GoalBase, self).__del__()
        except Exception as e:
            rhbplog.logerr("Error in destructor of GoalBase: %s", e)

    def pddlCallback(self, dummy):
        try:
            goalStatements = self.getGoalStatements()
            statePDDL = self.getStatePDDL()
            return GetPDDLResponse(**{"goalStatement": goalStatements,
                                      "stateStatement": statePDDL.statement,
                                      "statePredicates": list(statePDDL.predicates),
                                      "stateFunctions": list(statePDDL.functions)})
        except Exception:
            rhbplog.logerr("ROS service callback exception in 'pddlCallback' of goal '%s': %s", self._name,
                         traceback.format_exc())
            return None

    def getStatus(self, request):
        try:
            self.updateComputation(request.current_step)
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
        except Exception:
            rhbplog.logerr("ROS service callback exception in 'getStatus' of goal '%s': %s", self._name,
                         traceback.format_exc())
            return None


class OfflineGoal(AbstractGoalRepresentation):
    '''
    This class represents a goal, which is registered directly at the manager.
    As consequence no messages are used for communicating with the manager.
    '''

    def __init__(self, name, planner_prefix, permanent=False, conditions=None, priority=0, satisfaction_threshold=1.0):
        '''
        Constructor
        '''
        super(OfflineGoal, self).__init__(name=name, permanent=permanent,
                                          satisfaction_threshold=satisfaction_threshold, priority=priority)

        #nested goal object that takes care of calculation and provides management services
        self.__goal = Goal(name=name, satisfaction_threshold=satisfaction_threshold, planner_prefix=planner_prefix)

        if conditions is not None:
            for condition in conditions:
                self.add_condition(condition)

    def fetchStatus(self, current_step):
        self.__goal.updateComputation(current_step)
        self.fulfillment = self.__goal.computeSatisfaction()
        self.wishes = [Wish.from_wish_msg(wish) for wish in self.__goal.computeWishes()]
        self._priority = self.__goal.priority
        self.active = self.__goal.activated

    def fetchPDDL(self):
        goal_statements = self.__goal.getGoalStatements()
        state_pddl = self.__goal.getStatePDDL()
        return (PDDL(statement=goal_statements),
                PDDL(statement=state_pddl.statement, predicates=list(state_pddl.predicates), functions=list(
                    state_pddl.functions)))

    def add_condition(self, condition):
        self.__goal.addCondition(condition)

    def unregister(self):
        self.__goal._cleanup_topics_services()


class PublisherGoal(GoalBase):
    """
    Goal class which publishes its activation state as ROS topic
    """

    def __init__(self, name, permanent=False, conditions=None, plannerPrefix="", priority=0, satisfaction_threshold=1.0):
        """
        Without manual goal activation/deaction only permanent=False does make sense
        """
        super(PublisherGoal, self).__init__(name=name, permanent=permanent, conditions=conditions,
                                            plannerPrefix=plannerPrefix, priority=priority,
                                            satisfaction_threshold=satisfaction_threshold)

        self.__topic_name = self._service_prefix + "_activated"
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

    def _cleanup_topics_services(self):
        super(PublisherGoal, self)._cleanup_topics_services(terminate_services=True)
        self.__pub.charge()
