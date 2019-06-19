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
from rhbp_core.srv import AddGoal, GetStatus, GetStatusResponse, Enable, EnableResponse, GetPDDL, GetPDDLResponse, \
    SetInteger, SetIntegerResponse, RemoveGoal
from .activators import BooleanActivator
from behaviour_components.conditions import Condition
from .conditions import Conditonal
from .condition_elements import Wish
from .pddl import PDDL, mergeStatePDDL
from .sensors import TopicSensor
from utils.misc import FinalInitCaller
from utils.deprecation import deprecated
from utils.sensor_value_transformer import SensorValueTransformer


import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.goals')


class AbstractGoalRepresentation(object):
    '''
    This class represents a goal in the manager.
    '''

    __metaclass__ = ABCMeta

    def __init__(self, name, permanent=False, satisfaction_threshold=1.0, priority=0, active=True, enabled=True):
        self._name = name
        self._isPermanent = permanent
        self._priority = priority
        self._satisfaction_threshold = satisfaction_threshold
        self._wishes = []
        self._fulfillment = 0.0
        self._active = active  # This indicates (if True) that there have been no serious issues in the actual goal node
        # and the goal can be expected to be operational. If the actual goal reports _active == False
        # we will ignore it in activation computation.
        self._enabled = enabled  # True if the goal is not yet fulfilled. This member only exists as proxy for the
        # corresponding actual goal's property.
        # It is here because of the comprehensive status message, published each step by the manager for rqt

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
        return self._wishes

    @wishes.setter
    def wishes(self, wishes):
        '''
        :param wishes: list of (Wish())
        '''
        self._wishes = wishes

    @property
    def active(self):
        return self._active

    @active.setter
    def active(self, value):
        self._active = value

    @property
    def enabled(self):
        return self._enabled

    @enabled.setter
    def enabled(self, value):
        self._enabled = value

    @property
    def operational(self):
        """
        defines if the goal should be considered by the manager
        :return:
        """
        return self.active and self.enabled

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
    Therefore conditions can added, which are used for satisfaction computation
    '''

    SERVICE_NAME_ENABLE = "Enable"
    SERVICE_NAME_PRIORITY = "Priority"

    def __init__(self, name, planner_prefix, conditions=None, satisfaction_threshold=1.0, priority=0, active=True, enabled=True):
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
        self._enabled = enabled  # The enable Service sets the value of this property.
        self._priority = priority  # The higher the (unsigned) number the higher the importance
        self._services_running = False
        self._init_services()

    def _init_services(self):
        service_prefix = self._planner_prefix + '/' + self._name + '/'

        #these are convenience services which can be called remotely in order to configure currently enabled goals and priorities
        self._enable_service = rospy.Service(service_prefix + Goal.SERVICE_NAME_ENABLE, Enable, self._enable_callback)
        self._priorityService = rospy.Service(service_prefix + Goal.SERVICE_NAME_PRIORITY, SetInteger, self._set_priority_callback)
        self._services_running = True

    def _cleanup_topics_services(self):
        """
        Cleaning up ROS communication interface
        """
        if self._services_running:
            self._enable_service.shutdown()
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

    def _enable_callback(self, request):
        '''
        This method activates or deactivates the goal.
        This method must not block.
        '''
        self._enabled = request.active
        return EnableResponse()

    def _set_priority_callback(self, request):
        self._priority = request.value
        return SetIntegerResponse()

    @deprecated
    def addCondition(self, condition):
        self.add_condition(condition=condition)

    def add_condition(self, condition):
        '''
        This method adds a condition to the goal.
        It is not mandatory to use this method at all but it may make development easier because the default
        implementations of computeActivation(), computeSatisfaction(), and computeWishes work with the preconditions
        added here.
        If you don't want to use this mechanism then you HAVE TO implement those yourself!
        There is an AND relationship between all elements (all have to be fulfilled so that the behaviour is ready)
        To enable OR semantics use the Disjunction object.
        '''
        if issubclass(type(condition), Conditonal):
            self._conditions.append(condition)
        else:
            warnings.warn("That's no conditional object!")

    @property
    def enabled(self):
        return self._enabled

    @enabled.setter
    def enabled(self, enabled):
        self._enabled = enabled

    @property
    def priority(self):
        return self._priority

    @property
    def satisfaction_threshold(self):
        return self._satisfaction_threshold

    @property
    def active(self):
        return self._active

    @property
    def name(self):
        return self._name

    @property
    def conditions(self):
        return self._conditions


class GoalProxy(AbstractGoalRepresentation):
    '''
    This class connects a remote goal with the manager.
    It is instantiated automatically. Don't instantiated it manually
    '''

    SERVICE_TIMEOUT = 2
    MAX_CONSECUTIVE_TIMEOUTS = 10
    SERVICE_NAME_FETCH_PDDL = 'PDDL'
    SERVICE_NAME_GET_STATUS = 'GetStatus'

    def __init__(self, name, permanent, planner_prefix):
        '''
        Constructor
        '''
        super(GoalProxy, self).__init__(name, permanent)
        self._service_prefix = planner_prefix + '/' + self._name + '/'
        
        self.sensor_values = []
        self.__old_PDDL = (PDDL(statement=name), PDDL(statement="", predicates=[], functions=[]))
        self.__consecutive_timeouts = 0

    def fetchPDDL(self):
        '''
        This method fetches the PDDL from the actual goal node via GetPDDLservice call
        '''
        try:
            service_name = self._service_prefix + GoalProxy.SERVICE_NAME_FETCH_PDDL
            rhbplog.logdebug("Waiting for service %s", service_name)
            rospy.wait_for_service(service_name, timeout=self.SERVICE_TIMEOUT)
            self.__consecutive_timeouts = 0
        except rospy.ROSInterruptException:  # ros shutdown
            self._handle_service_timeout(logging_enabled=False)
            return self.__old_PDDL
        except rospy.ROSException:
            self._handle_service_timeout(logging_enabled=True)
            return self.__old_PDDL
        try:
            getPDDLRequest = rospy.ServiceProxy(service_name, GetPDDL)
            pddl = getPDDLRequest()
            self.__old_PDDL = (PDDL(statement=pddl.goalStatement),
                    PDDL(statement=pddl.stateStatement, predicates=pddl.statePredicates,
                         functions=pddl.stateFunctions))
            return self.__old_PDDL
        except rospy.ServiceException:
            rhbplog.logerr("ROS service exception in 'fetchPDDL' of goal '%s': %s", self._name,
                         traceback.format_exc())
            return self.__old_PDDL

    def fetchStatus(self, current_step):
        """
        This method fetches the status from the actual goal node via GetStatus service call
        :param current_step: 
        """

        try:
            service_name = self._service_prefix + GoalProxy.SERVICE_NAME_GET_STATUS
            rhbplog.logdebug("Waiting for service %s", service_name)
            rospy.wait_for_service(service_name, timeout=self.SERVICE_TIMEOUT)
            self.__consecutive_timeouts = 0
        except rospy.ROSInterruptException:  # ros shutdown
            self._handle_service_timeout(logging_enabled=False)
            return
        except rospy.ROSException:
            self._handle_service_timeout(logging_enabled=True)
            return

        try:
            get_status_request = rospy.ServiceProxy(service_name, GetStatus)
            status = get_status_request(current_step).status
            self._fulfillment = status.satisfaction
            self._wishes = [Wish.from_wish_msg(wish) for wish in status.wishes]
            self._active = status.active
            self._enabled = status.enabled
            self._priority = status.priority
            self._satisfaction_threshold = status.threshold
            self.sensor_values = status.sensor_values
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

    def _handle_service_timeout(self, logging_enabled=True):
        """
        basically disable the goal in case a service has timeout
        """
        if logging_enabled:
            rhbplog.logerr("ROS service timeout of goal '%s': %s.", self._name,
                         traceback.format_exc())

        self.__consecutive_timeouts += 1

        if self.__consecutive_timeouts > self.MAX_CONSECUTIVE_TIMEOUTS:
            if logging_enabled:
                rhbplog.logerr("Too many consecutive timeouts for goal '%s'! Fulfillment will be reset", self._name)
            self._active = False
            self.fulfillment = 0.0

    @AbstractGoalRepresentation.enabled.setter
    def enabled(self, value):
        # inform remote goal about new enabled state
        service_name = self._service_prefix + Goal.SERVICE_NAME_ENABLE
        rhbplog.logdebug("Waiting for service %s", service_name)
        try:
            rospy.wait_for_service(service_name, timeout=self.SERVICE_TIMEOUT)
            self.__consecutive_timeouts = 0
        except rospy.ROSInterruptException:  # ros shutdown
            self._handle_service_timeout(logging_enabled=False)
            return
        except rospy.ROSException:
            self._handle_service_timeout(logging_enabled=True)
            return
        try:
            enable_request = rospy.ServiceProxy(service_name, Enable)
            enable_request(value)
            self._enabled = value
        except rospy.ServiceException:
            rhbplog.logerr("ROS service exception in 'enabled' of goal '%s': %s", self._name,
                         traceback.format_exc())


# TODO Rename to RemoteGoal
class GoalBase(Goal):
    """
    Goal, which is automatically registered at the manager. Therefore the ros messages are used.
    Allows to be on different node than the manager.
    """

    __metaclass__ = FinalInitCaller

    SERVICE_TIMEOUT = 5

    def __init__(self, name, permanent=False, conditions=None, planner_prefix="", priority=0, satisfaction_threshold=1.0,
                 enabled=True):
        """

        :param name:  a unique name is mandatory
        :param permanent:
        :param conditions:
        :param planner_prefix: if you have multiple planners in the same ROS environment use a prefix to identify the right one.
        :param priority:
        :param satisfaction_threshold:
        :param enabled:
        """
        super(GoalBase, self).__init__(name=name, planner_prefix=planner_prefix, conditions=conditions,
                                       satisfaction_threshold=satisfaction_threshold, priority=priority,
                                       enabled=enabled)
        self._name = name

        self._permanent = permanent
        self._planner_prefix = planner_prefix
        self._registered = False  # keeps track of goal registration state
        self._sensor_transformer = SensorValueTransformer()

    def _init_services(self):
        super(GoalBase, self)._init_services()
        self._service_prefix = self._planner_prefix + '/' + self._name + '/'
        self._getStatusService = rospy.Service(self._service_prefix + GoalProxy.SERVICE_NAME_GET_STATUS, GetStatus,
                                               self._get_status_callback)
        self._pddlService = rospy.Service(self._service_prefix + GoalProxy.SERVICE_NAME_FETCH_PDDL, GetPDDL,
                                          self._pddl_callback)

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
            rhbplog.logwarn("Goal '%s' is already registered", self._name)
            return
        try:

            if not self._services_running:  # (re)register services if necessary
                self._init_services()

            service_name = self._planner_prefix + '/' + 'AddGoal'

            service_found = False
            while not service_found:
                try:
                    rospy.wait_for_service(service_name, timeout=self.SERVICE_TIMEOUT)
                    service_found = True
                except rospy.ROSInterruptException:
                    rhbplog.loginfo("Stopping registration of %s, system is shutting down.", self._name)
                    return
                except rospy.ROSException:
                    rhbplog.logwarn("Goal '%s': Registration timeout for service '%s'. Keep waiting... Please check if "
                                    "you use the correct 'planner_prefix'. Current prefix:'%s'", self._name,
                                    service_name, self._planner_prefix)

            add_goal = rospy.ServiceProxy(service_name, AddGoal)
            add_goal(self._name, self._permanent)
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
        if hasattr(self, '_getStatusService') and self._getStatusService:
            self._getStatusService.shutdown()
        if hasattr(self, '_pddlService') and self._pddlService:
            self._pddlService.shutdown()

    def unregister(self, terminate_services=True):
        """
        Remove/Unregister behaviour from the manager
        :param terminate_services: True for shutting down all service interfaces as well
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
            rhbplog.logwarn("Goal: %s unregister() failed.", self._name)

        self._registered = False
        if terminate_services:
            self._cleanup_topics_services()

    def __del__(self):
        '''
        Destructor
        '''
        try:
            if hasattr(self, '_registered') and self._registered:
                self.unregister(terminate_services=True)

            super(GoalBase, self).__del__()
        except Exception as e:
            rhbplog.logerr("Error in destructor of GoalBase: %s", e)

    def _pddl_callback(self, dummy):
        try:
            goalStatements = self.getGoalStatements()
            statePDDL = self.getStatePDDL()
            return GetPDDLResponse(**{"goalStatement": goalStatements,
                                      "stateStatement": statePDDL.statement,
                                      "statePredicates": list(statePDDL.predicates),
                                      "stateFunctions": list(statePDDL.functions)})
        except Exception:
            rhbplog.logerr("ROS service callback exception in '_pddl_callback' of goal '%s': %s", self._name,
                         traceback.format_exc())
            return None

    def _get_status_callback(self, request):

        try:
            self.updateComputation(request.current_step)
            
            sensor_values = self._sensor_transformer.get_sensor_values(self._conditions)
            
            status = Status(**{
                "name": self._name,
                "satisfaction": self.computeSatisfaction(),
                "wishes": self.computeWishes(),
                "active": self._active,
                "enabled": self._enabled,
                "priority": self._priority,
                "threshold": self._satisfaction_threshold,
                "sensor_values": sensor_values
            })
            return GetStatusResponse(status)
        except Exception:
            rhbplog.logerr("ROS service callback exception in '_get_status_callback' of goal '%s': %s", self._name,
                         traceback.format_exc())
            return None

    @property
    def isPermanent(self):
        return self._permanent


class OfflineGoal(AbstractGoalRepresentation):
    '''
    This class represents a goal, which is registered directly at the manager.
    As consequence no messages are used for communicating with the manager.
    '''

    def __init__(self, name, planner_prefix, permanent=False, conditions=None, priority=0, satisfaction_threshold=1.0,
                 enabled=True):
        '''
        Constructor
        '''
        super(OfflineGoal, self).__init__(name=name, permanent=permanent,
                                          satisfaction_threshold=satisfaction_threshold, priority=priority,
                                          enabled=enabled)

        #nested goal object that takes care of calculation and provides management services
        self.__goal = Goal(name=name, satisfaction_threshold=satisfaction_threshold, planner_prefix=planner_prefix)

        if conditions is not None:
            for condition in conditions:
                self.add_condition(condition)

    def fetchStatus(self, current_step):
        self.__goal.updateComputation(current_step)
        self._fulfillment = self.__goal.computeSatisfaction()
        self._wishes = [Wish.from_wish_msg(wish) for wish in self.__goal.computeWishes()]
        self._priority = self.__goal.priority
        self._active = self.__goal.active
        self._enabled = self.__goal.enabled
        self._satisfaction_threshold = self.__goal.satisfaction_threshold

    def fetchPDDL(self):
        goal_statements = self.__goal.getGoalStatements()
        state_pddl = self.__goal.getStatePDDL()
        return (PDDL(statement=goal_statements),
                PDDL(statement=state_pddl.statement, predicates=list(state_pddl.predicates), functions=list(
                    state_pddl.functions)))

    def add_condition(self, condition):
        self.__goal.add_condition(condition)

    def unregister(self):
        self.__goal._cleanup_topics_services()


class PublisherGoal(GoalBase):
    """
    Goal class which publishes its 'enabled' state as ROS topic
    """

    def __init__(self, name, permanent=False, conditions=None, planner_prefix="", priority=0, satisfaction_threshold=1.0,
                 enabled=True):
        """
        Without manual goal enabling/disabling only permanent=False does make sense
        """
        super(PublisherGoal, self).__init__(name=name, permanent=permanent, conditions=conditions,
                                            planner_prefix=planner_prefix, priority=priority,
                                            satisfaction_threshold=satisfaction_threshold, enabled=enabled)

        self.__topic_name = self._service_prefix + "_enabled"
        self.__pub = rospy.Publisher(self.__topic_name, Bool, queue_size=1)
        self.__condition = None

    def updateComputation(self, manager_step):
        super(PublisherGoal, self).updateComputation(manager_step)
        self.__pub.publish(self._enabled)

    def create_condition(self):
        """
        Creating a new condition object based on the positive goal enabled state --> full activation on enabled goal
        :returns new condition object
        """
        if not self.__condition:
            condition_name = self._name + "_condition"
            sensor_name = self._name + "_sensor"
            sensor = TopicSensor(name=sensor_name, topic=self.__topic_name, message_type=Bool,
                                 initial_value=self._enabled)
            activator = BooleanActivator(desiredValue=True)
            self.__condition = Condition(name=condition_name, sensor=sensor, activator=activator)
        return self.__condition

    def _cleanup_topics_services(self):
        super(PublisherGoal, self)._cleanup_topics_services()
        if hasattr(self, "__pub") and self.__pub:
            self.__pub.unregister()
