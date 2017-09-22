'''
Created on 23.04.2015

@author: wypler,hrabia
'''

import rospy
import itertools
from std_srvs.srv import Empty, EmptyResponse
from rhbp_core.msg import PlannerStatus, Status, Correlation, Wish, DiscoverInfo
from rhbp_core.srv import AddBehaviour, AddBehaviourResponse, AddGoal, AddGoalResponse, RemoveBehaviour, RemoveBehaviourResponse, RemoveGoal, RemoveGoalResponse, ForceStart, ForceStartResponse, Activate
from .behaviours import Behaviour
from .goals import GoalProxy
from .pddl import PDDL, mergeStatePDDL, tokenizePDDL, getStatePDDLchanges, predicateRegex, init_missing_functions, create_valid_pddl_name
from .planner import MetricFF
from .activation_algorithm import ActivationAlgorithmFactory
from utils.misc import LogFileWriter

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.planning')

import sys
import threading

class Manager(object):

    USE_ONLY_RUNNING_BEHAVIOURS_FOR_INTERRUPTIBLE_DEFAULT_VALUE = False

    MANAGER_DISCOVERY_TOPIC = "rhbp_discover"

    '''
    This is the manager class that keeps track of all elements in the network (behaviours, goals, sensors).
    Behaviours need this to know what sensors exist in the world and how they are correlated their measurement.
    They also need to know the goals to decide whether they are supporting them (which increases their activation) or inhibiting them (which decreases their activation).
    And last, they need to know whether another behaviour conflicts with them to reduce the activation of it.
    Also global constants like activation thresholds are stored here.
    '''

    def __init__(self, activated = True, use_only_running_behaviors_for_interRuptible = USE_ONLY_RUNNING_BEHAVIOURS_FOR_INTERRUPTIBLE_DEFAULT_VALUE, **kwargs):
        '''
        Constructor
        '''
        self._prefix = kwargs["prefix"] if "prefix" in kwargs else "" # if you have multiple planners in the same ROS environment use this to distinguish between the instances
        self._sensors = [] #TODO this is actually not used at all in the moment, only behaviour know the sensors and activators
        self._goals = []
        self._activeGoals = [] # pre-computed (in step()) list of operational goals
        self._behaviours = []
        self._activeBehaviours = [] # pre-computed (in step()) list of operational behaviours
        self._totalActivation = 0.0 # pre-computed (in step()) sum all activations of active behaviours
        self._activationThreshold = kwargs["activationThreshold"] if "activationThreshold" in kwargs \
            else rospy.get_param("~activationThreshold", 7.0) # not sure how to set this just yet.
        self.__activationDecay = kwargs["activationDecay"] if "activationDecay" in kwargs else None
        self._create_log_files = kwargs["createLogFiles"] if "createLogFiles" in kwargs else rospy.get_param(
            "~createLogFiles", False)  # not sure how to set this just yet.
        #configures if all contained behaviour or only the executed behaviours are used to determine if the manager is interruptable
        self.__use_only_running_behaviors_for_interruptible = use_only_running_behaviors_for_interRuptible

        self.__conflictor_bias = kwargs['conflictorBias'] if 'conflictorBias' in kwargs else None
        self.__goal_bias = kwargs['goalBias'] if 'goalBias' in kwargs else None
        self.__predecessor_bias = kwargs['predecessorBias'] if 'predecessorBias' in kwargs else None
        self.__successor_bias = kwargs['successorBias'] if 'successorBias' in kwargs else None
        self.__plan_bias = kwargs['planBias'] if 'planBias' in kwargs else None
        self.__situation_bias = kwargs['situationBias'] if 'situationBias' in kwargs else None

        self.__max_parallel_behaviours = kwargs['max_parallel_behaviours'] if 'max_parallel_behaviours' in kwargs else \
            rospy.get_param("~max_parallel_behaviours", sys.maxint)

        self._stepCounter = 0

        self._step_lock = threading.Lock()

        self.__log_file_path_prefix = self._prefix + '/' if self._prefix else ''

        if self._create_log_files:
            self.__threshFile = LogFileWriter(path=self.__log_file_path_prefix, filename="threshold", extension=".log")
            self.__threshFile.write("{0}\t{1}\n".format("Time", "activationThreshold"))

        self.__replanningNeeded = False # this is set when behaviours or goals are added or removed, or the last planning attempt returned an error.
        self.__previousStatePDDL = PDDL()
        self.__sensorChanges = {} # this dictionary stores all sensor changes between two steps in the form {sensor name <string> : indicator <float>}. Note: the float is not scaled [-1.0 to 1.0] but shows a direction (positive or negative).
        self._plan = {}
        self._planExecutionIndex = 0
        self.__goalPDDLs = {}

        self.planner = MetricFF()

        #create activation algorithm
        algorithm_name = kwargs['activation_algorithm'] if 'activation_algorithm' in kwargs else 'default'
        rhbplog.loginfo("Using activation algorithm: %s", algorithm_name)
        self.activation_algorithm = ActivationAlgorithmFactory.create_algorithm(algorithm_name, self)
        #trigger update once in order to initialize algorithm properly
        self._update_bias_parameters()

        self.pause_counter = 0  # counts pause requests, step is only executed at pause_counter = 0
        self.__activated = activated

        self.init_services_topics()

        self.__executedBehaviours = []

    def init_services_topics(self):
        self._service_prefix = self._prefix + '/'
        self.__addBehaviourService = rospy.Service(self._service_prefix + 'AddBehaviour', AddBehaviour,
                                                   self.__add_behaviour_callback)
        self.__addGoalService = rospy.Service(self._service_prefix + 'AddGoal', AddGoal, self.__add_goal_callback)
        self.__removeBehaviourService = rospy.Service(self._service_prefix + 'RemoveBehaviour', RemoveBehaviour,
                                                      self.__remove_behaviour_callback)
        self.__removeGoalService = rospy.Service(self._service_prefix + 'RemoveGoal', RemoveGoal, self.__remove_goal_callback)
        self.__manualStartService = rospy.Service(self._service_prefix + 'ForceStart', ForceStart, self.__manual_start_callback)
        self.__pauseService = rospy.Service(self._service_prefix + 'Pause', Empty, self.__pause_callback)
        self.__resumeService = rospy.Service(self._service_prefix + 'Resume', Empty, self.__resume_callback)
        self.__statusPublisher = rospy.Publisher(self._service_prefix + 'Planner/plannerStatus', PlannerStatus,
                                                 queue_size=1, latch=True)

        self.__pub_discover = rospy.Publisher(name=Manager.MANAGER_DISCOVERY_TOPIC, data_class=DiscoverInfo, queue_size=1)

    def __del__(self):
        self.__addBehaviourService.shutdown()
        self.__addGoalService.shutdown()
        self.__removeBehaviourService.shutdown()
        self.__removeGoalService.shutdown()
        self.__manualStartService.shutdown()
        self.__statusPublisher.unregister()

    def _getDomainName(self):
        return create_valid_pddl_name(self._prefix) if self._prefix else "UNNAMED"

    def _fetchPDDL(self):
        '''
        This method fetches the PDDL from all behaviours and goals, merges the state descriptions and returns a tuple of
        (domainPDDL, problemPDDL) strings ready for ff.
        As a side effect, is also computes the sensor changes that happened between the last invocation and now.
        '''
        behaviourPDDLs = [behaviour.fetchPDDL() for behaviour in self._activeBehaviours]
        self.__goalPDDLs = {goal: goal.fetchPDDL() for goal in self._activeGoals}
        pddl = PDDL()
        #Get relevant domain information from behaviour pddls
        for actionPDDL, statePDDL in behaviourPDDLs:
            pddl.statement += actionPDDL.statement
            pddl.predicates = pddl.predicates.union(actionPDDL.predicates)
            pddl.functions = pddl.functions.union(actionPDDL.functions)
            pddl.functions = pddl.functions.union(statePDDL.functions)
        # Get relevant domain information from goal pddls
        for goal, goal_pddl in self.__goalPDDLs.iteritems():
            if goal_pddl is None:
                continue
            # pddl.statement and  pddl.predicates are not needed from goals for the domain description
            actionPDDL, statePDDL = goal_pddl
            pddl.functions = pddl.functions.union(actionPDDL.functions)
            pddl.functions = pddl.functions.union(statePDDL.functions)
        domainPDDLString = "(define (domain {0})\n".format(self._getDomainName())
        #Update requirements if necessary
        #Actually :fluents could just be :numeric-fluents, but this is not accepted by metric-ff
        domainPDDLString += "(:requirements :strips :adl :equality :negation :conditional-effects :fluents)\n"
        domainPDDLString += "(:predicates\n    " + "\n    ".join("({0})".format(x) for x in pddl.predicates) + ")\n"
        domainPDDLString += "(:functions\n    " + "\n    ".join("({0})".format(x) for x in pddl.functions) + ")\n"
        domainPDDLString += pddl.statement + ")"
        mergedStatePDDL = PDDL()
        for _actionPDDL, statePDDL in behaviourPDDLs:
            mergedStatePDDL = mergeStatePDDL(statePDDL, mergedStatePDDL)
        for v in self.__goalPDDLs.itervalues():
            if not v is None:
                _goalPDDL, statePDDL = v
                mergedStatePDDL = mergeStatePDDL(statePDDL, mergedStatePDDL)

        mergedStatePDDL = init_missing_functions(pddl, mergedStatePDDL)

        # filter out negative predicates. FF can't handle them!
        statePDDL = PDDL(statement = "\n\t\t".join(
            filter(lambda x: predicateRegex.match(x) is None or predicateRegex.match(x).group(2) is None, tokenizePDDL(mergedStatePDDL.statement)))
        )# if the regex does not match it is a function (which is ok) and if the second group is None it is not negated (which is also ok)

        if self._create_log_files:
            domainLog = LogFileWriter(path=self.__log_file_path_prefix, filename="pddl{0}Domain".format(self._stepCounter), extension=".pddl" )
            domainLog.write(domainPDDLString)

        # compute changes
        self.__sensorChanges = getStatePDDLchanges(self.__previousStatePDDL, statePDDL)
        self.__previousStatePDDL = statePDDL
        return domainPDDLString
    
    def _create_problem_pddl(self, goals):
        '''
        This method creates the problem PDDL for a given set of goals.
        It relies on the fact that self.fetchPDDL() has run before and filled the self.__goalPDDLs dictionary with the most recent responses from the actual goals and self.__previousStatePDDL with the CURRENT state PDDL
        '''
        goalConditions = (self.__goalPDDLs[goal][0].statement for goal in goals) # self.__goalPDDLs[goal][0] is the goalPDDL of goal's (goalPDDL, statePDDL) tuple
        problemPDDLString = "(define (problem problem-{0})\n\t(:domain {0})\n\t(:init \n\t\t{1}\n\t)\n".format(self._getDomainName(), self.__previousStatePDDL.statement) # at this point the "previous" is the current state PDDL
        problemPDDLString += "\t(:goal (and {0}))\n\t(:metric minimize (costs))\n".format(" ".join(goalConditions))
        problemPDDLString += ")\n"

        if self._create_log_files:
            filename = "pddl{0}Problem_{1}".format(self._stepCounter, ''.join((str(g) for g in goals)))
            domainLog = LogFileWriter(path=self.__log_file_path_prefix, filename=filename, extension=".pddl" )
            domainLog.write(problemPDDLString)

        return problemPDDLString
    
    def _generate_priority_goal_sequences(self):
        '''
        This is a generator that generates goal sequences with descending priorities.
        It yields sorted lists with the most important goal at the front and strips away one element from the back at each iteration.
        After the most important goal was the only remaining element in the list the same process repeats for the second most important goals and so on.
        '''
        sortedGoals = sorted(self._activeGoals, key=lambda x: x.priority, reverse = True)
        numElements = len(sortedGoals)
        for i in xrange(0, numElements, 1):
            for j in xrange(numElements, i, -1):
                yield sortedGoals[i : j]
    
    def _plan_if_necessary(self):
        '''
        this method plans using the symbolic planner it it is required to do so.
        Replanning is required whenever any or many of the following conditions is/are met:
        1) Behaviours or Goals have been added or removed
        2) The state of the world has changed an this change is not caused by the next behaviour in the plan
        3) An unexpected behaviour was running (not head of the plan or flagged as independentFromPlanner)
        4) An interruptable behaviour timed out
        5) The last planning attempt was unsuccessful
        '''

        if rospy.get_param("~planBias", 1.0) == 0.0:
            return # return if planner is disabled

        domainPDDL = self._fetchPDDL() # this also updates our self.__sensorChanges and self.__goalPDDLs dictionaries
        # now check whether we expected the world to change so by comparing the observed changes to the correlations of the running behaviours
        changesWereExpected = True
        for sensorName, indicator in self.__sensorChanges.iteritems():
            changeWasExpected = False
            for behaviour in self.__executedBehaviours:
                for item in behaviour.correlations:
                    if item.get_pddl_effect_name() == sensorName and item.indicator * indicator > 0: # the observed change happened because of the running behaviour (at least the behaviour is correlated to the changed sensor in the correct way)
                        changeWasExpected = True
                        break
                if changeWasExpected:
                    break
            if not changeWasExpected:
                changesWereExpected = False
                break
        # the next part is a little plan execution monitoring
        # it tracks progress on the plan and finds out if something unexpected finished.
        unexpectedBehaviourFinished = False
        # make sure the finished behaviour was part of the plan at all (otherwise it is unexpected)
        if self._plan:
            for behaviour in self.__executedBehaviours:
                if behaviour.justFinished and not behaviour.independentFromPlanner and behaviour.name not in [name for index, name in self._plan["actions"].iteritems() if index >= self._planExecutionIndex]:
                    unexpectedBehaviourFinished = True # it was unexpected
                    break
            if not unexpectedBehaviourFinished: # if we found a behaviour that executed that was not part of the plan (and not flagged independentromPlan) we can stop here, otherwise we have to ensure that the behaviours finished in correct order.
                for index in filter(lambda x: x >= self._planExecutionIndex, sorted(self._plan["actions"].keys())): # walk along the remaining plan
                    for behaviour in self.__executedBehaviours: # only those may be finished. the others were not even running
                        if behaviour.name == self._plan["actions"][index] and behaviour.justFinished:  # inspect the behaviour at the current index of the plan
                            if self._planExecutionIndex == index: # if it was a planned behaviour and it finished
                                self._planExecutionIndex += 1 # we are one step ahead in our plan
                            elif not behaviour.independentFromPlanner: # otherwise and if the behaviour was not allowed to act reactively on its own (flagged as independentFromPlanner)
                                unexpectedBehaviourFinished = True # it was unexpected
        # now, we know whether we need to plan again or not
        if self.__replanningNeeded or unexpectedBehaviourFinished or not changesWereExpected:
            rhbplog.loginfo("### PLANNING ### because\nreplanning was needed: %s\nchanges were unexpected: %s\nunexpected behaviour finished: %s", self.__replanningNeeded, not changesWereExpected, unexpectedBehaviourFinished)
            # now we need to make the best of our planning problem:
            # In cases where the full set of goals can't be reached because the planner does not find a solution a reduced set should be used.
            # The reduction will eliminate goals of inferiour priority until the highest priority goal is tried alone.
            # If that cannot be reached the search goes backwards and tries all other goals with lower priorities in descending order until a reachable goal is found.
            for goalSequence in self._generate_priority_goal_sequences():
                try:
                    rhbplog.logdebug("trying to reach goals %s", goalSequence)
                    problemPDDL = self._create_problem_pddl(goalSequence)
                    tmpPlan = self.planner.plan(domainPDDL, problemPDDL)
                    if tmpPlan and "cost" in tmpPlan and tmpPlan["cost"] != -1.0:
                        rhbplog.loginfo("FOUND PLAN: %s", tmpPlan)
                        self._plan = tmpPlan
                        self.__replanningNeeded = False
                        self._planExecutionIndex = 0
                        break
                    else:
                        rhbplog.loginfo("PROBLEM IMPOSSIBLE")
                except Exception as e:
                    rhbplog.logerr("PLANNER ERROR: %s", e)
                    self.__replanningNeeded = True # in case of planning exceptions try again next iteration
        else:
            rhbplog.loginfo("### NOT PLANNING: replanning was needed: %s;changes were unexpected: %s;unexpected behaviour finished: %s; current plan execution index: %s", self.__replanningNeeded, not changesWereExpected, unexpectedBehaviourFinished, self._planExecutionIndex)
    
    def send_discovery(self):
        """
        Send manager/planner discovery message
        """
        msg = DiscoverInfo()
        msg.stamp = rospy.Time.now()
        msg.manager_prefix = self._prefix
        self.__pub_discover.publish(msg)

    def step(self):
        if (self.pause_counter > 0) or (not self.__activated):
            return

        if self._create_log_files:  # debugging only
            self.__threshFile.append("{0:f}\t{1:f}\n".format(rospy.get_time(), self._activationThreshold))

        self.send_discovery()

        with self._step_lock:
            rhbplog.logdebug("###################################### STEP {0} ######################################".format(self._stepCounter))

            self.update_activation()

            rhbplog.loginfo("############## ACTIONS ###############")
            self.__executedBehaviours = filter(lambda x: x.isExecuting, self._behaviours) # actually, activeBehaviours should be enough as search space but if the behaviour implementer resets active before isExecuting we are safe this way
            amount_started_behaviours = 0
            amount_currently_selected_behaviours = 0

            rhbplog.loginfo("currently running behaviours: %s", self.__executedBehaviours)

            currently_influenced_sensors = set()

            # perform the decision making based on the calculated activations
            for behaviour in sorted(self._behaviours, key = lambda x: x.activation, reverse = True):
                ### now comes a series of tests that a behaviour must pass in order to get started ###
                if not behaviour.active and not behaviour.manualStart: # it must be active
                    rhbplog.loginfo("'%s' will not be started because it is not active", behaviour.name)
                    continue
                if behaviour.isExecuting: # it must not already run
                    #It is important to remember that only interruptable behaviours can be stopped by the manager
                    if behaviour.executionTimeout != -1 and behaviour.executionTime >= behaviour.executionTimeout \
                            and behaviour.interruptable:
                        rhbplog.loginfo("STOP BEHAVIOUR '%s' because it timed out", behaviour.name)
                        self._stop_behaviour(behaviour, True)
                        self.__replanningNeeded = True # this is unusual so replan
                    elif not behaviour.executable and behaviour.interruptable:
                        rhbplog.loginfo("STOP BEHAVIOUR '%s' because it is not executable anymore", behaviour.name)
                        self._stop_behaviour(behaviour, True)
                    elif behaviour.activation < self._activationThreshold and behaviour.interruptable:
                        rhbplog.loginfo("STOP BEHAVIOUR '%s' because of too low activation %f < %f", behaviour.name, behaviour.activation, self._activationThreshold )
                        self._stop_behaviour(behaviour, False)
                    elif self.__max_parallel_behaviours > 0 and amount_currently_selected_behaviours >= self.__max_parallel_behaviours:
                        rhbplog.loginfo("STOP BEHAVIOUR '%s' because of too many executed behaviours", behaviour.name)
                        self._stop_behaviour(behaviour, False)
                    else:
                        rhbplog.loginfo("'%s' will not be started because it is already executing", behaviour.name)
                        behaviour.executionTime += 1
                        amount_currently_selected_behaviours +=1
                        if behaviour.requires_execution_steps:
                            behaviour.do_step()
                    continue
                if not behaviour.executable and not behaviour.manualStart: # it must be executable
                    rhbplog.loginfo("%s will not be started because it is not executable", behaviour.name)
                    continue
                if behaviour.activation < self._activationThreshold and not behaviour.manualStart: # it must have high-enough activation
                    rhbplog.loginfo("%s will not be started because it has not enough activation (%f < %f)", behaviour.name, behaviour.activation, self._activationThreshold)
                    continue

                currently_influenced_sensors = self._get_currently_influenced_sensors()

                behaviour_is_interferring_others, currently_influenced_sensors = self.handle_interferring_correlations(behaviour, currently_influenced_sensors)

                if behaviour_is_interferring_others:
                    continue

                # Do not execute more behaviours than allowed, behaviours are ordered by decending activation hence
                # we prefer behaviours with higher activation
                if self.__max_parallel_behaviours > 0 and amount_currently_selected_behaviours >= self.__max_parallel_behaviours:
                    continue

                ### if the behaviour got here it really is ready to be started ###
                rhbplog.loginfo("START BEHAVIOUR %s", behaviour.name)
                if behaviour.manualStart:
                    rhbplog.loginfo("BEHAVIOUR %s WAS STARTED BECAUSE OF MANUAL REQUEST")
                behaviour.start()
                amount_currently_selected_behaviours +=1

                self.__executedBehaviours.append(behaviour)
                amount_started_behaviours += 1
                rhbplog.loginfo("now running behaviours: %s", self.__executedBehaviours)

            activation_threshold_decay = rospy.get_param("~activationThresholdDecay", .8)

            self._publish_planner_status(activation_threshold_decay, currently_influenced_sensors)

            # Reduce or increase the activation threshold based on executed and started behaviours
            if len(self.__executedBehaviours) == 0 and len(self._activeBehaviours) > 0:
                self._activationThreshold *= activation_threshold_decay
                rhbplog.loginfo("REDUCING ACTIVATION THRESHOLD TO %f", self._activationThreshold)
            elif amount_started_behaviours > 0:
                rhbplog.loginfo("INCREASING ACTIVATION THRESHOLD TO %f", self._activationThreshold)
                self._activationThreshold *= (1 / activation_threshold_decay)

        self._stepCounter += 1

    def _publish_planner_status(self, activation_threshold_decay, currently_influenced_sensors):
        """
        Collect all information for the plannerStatusMessage and publish it
        :param activation_threshold_decay: current activationThresholdDecay
        :param currently_influenced_sensors: set() of currently influenced sensors
        """
        if self.__statusPublisher.get_num_connections() == 0:
            return

        plannerStatusMessage = PlannerStatus()
        plannerStatusMessage.activationThreshold = self._activationThreshold
        for goal in self._goals:
            statusMessage = Status()
            statusMessage.name = goal.name
            statusMessage.wishes = [w.get_wish_msg() for w in goal.wishes]
            statusMessage.active = goal.active
            statusMessage.activated = goal.activated
            statusMessage.satisfaction = goal.fulfillment
            statusMessage.priority = goal.priority
            plannerStatusMessage.goals.append(statusMessage)

        # collect all that stuff for the rqt gui
        for behaviour in self._behaviours:
            statusMessage = Status()
            statusMessage.name = behaviour.name
            statusMessage.activation = behaviour.activation
            statusMessage.satisfaction = behaviour.preconditionSatisfaction
            statusMessage.isExecuting = behaviour.isExecuting
            statusMessage.executionTimeout = behaviour.executionTimeout
            statusMessage.executionTime = behaviour.executionTime
            statusMessage.progress = behaviour.progress
            statusMessage.executable = behaviour.executable
            statusMessage.threshold = behaviour.readyThreshold
            statusMessage.priority = behaviour.priority
            statusMessage.interruptable = behaviour.interruptable
            statusMessage.independentFromPlanner = behaviour.independentFromPlanner
            statusMessage.activated = behaviour.activated
            statusMessage.active = behaviour.active
            statusMessage.correlations = [correlation.get_msg() for correlation in behaviour.correlations]
            statusMessage.wishes = [w.get_wish_msg() for w in behaviour.wishes]
            plannerStatusMessage.behaviours.append(statusMessage)
        plannerStatusMessage.runningBehaviours = map(lambda x: x.name, self.__executedBehaviours)
        plannerStatusMessage.influencedSensors = list(currently_influenced_sensors)
        plannerStatusMessage.activationThresholdDecay = activation_threshold_decay
        plannerStatusMessage.stepCounter = self._stepCounter
        plannerStatusMessage.plan = self._plan
        plannerStatusMessage.plan_index = self._planExecutionIndex
        self.__statusPublisher.publish(plannerStatusMessage)

    def update_activation(self, plan_if_necessary=True):
        """
        Update all information about behaviours and goals and update the activation calculation
        :param plan_if_necessary: enable or disable the check for required planning
        """
        self._totalActivation = 0.0
        ### collect information about behaviours ###
        for behaviour in self._behaviours:
            behaviour.fetchStatus(self._stepCounter)
            if behaviour.active:
                self._totalActivation += behaviour.activation
        if self._totalActivation == 0.0:
            self._totalActivation = 1.0  # the behaviours are going to divide by this so make sure it is non-zero
        rhbplog.logdebug("############# GOAL STATES #############")
        ### collect information about goals ###
        for goal in self._goals:
            goal.fetchStatus(self._stepCounter)
            rhbplog.logdebug("%s: active: %s, fulfillment: %f, wishes %s", goal.name, goal.active, goal.fulfillment,
                             goal.wishes)
            # Deactivate non-permanent and satisfied goals
            if goal.active and not goal.isPermanent and goal.satisfied:
                goal.activated = False
                rhbplog.logdebug("Set Activated of %s goal to False", goal.name)
                goal.active = False  # this needs to be set locally because the effect of the Activate service call above is only "visible" by GetStatus service calls in the future but we need it to be deactivated NOW
        ### do housekeeping ###
        self._activeGoals = filter(lambda x: x.active, self._goals)
        self._activeBehaviours = filter(lambda x: x.active,
                                        self._behaviours)  # this line (and the one above) must happen BEFORE computeActivation() of the behaviours is called in each step.
        ### use the symbolic planner if necessary ###
        if plan_if_necessary:
            self._plan_if_necessary()
        self._update_bias_parameters()
        ### log behaviour stuff ###
        rhbplog.logdebug("########## BEHAVIOUR  STATES ##########")
        for behaviour in self._behaviours:
            ### do the activation computation ###
            self.activation_algorithm.compute_behaviour_activation_step(ref_behaviour=behaviour)
            rhbplog.logdebug("%s", behaviour.name)
            rhbplog.logdebug("\tactive %s", behaviour.active)
            rhbplog.logdebug("\twishes %s", behaviour.wishes)
            rhbplog.logdebug(
                "\texecutable: {0} ({1})\n".format(behaviour.executable, behaviour.preconditionSatisfaction))

        ### commit the activation computed in this step ###
        for behaviour in self._behaviours:
            self.activation_algorithm.commit_behaviour_activation(ref_behaviour=behaviour)
            rhbplog.logdebug("activation of %s after this step: %f", behaviour.name, behaviour.activation)
        rhbplog.loginfo("current activation threshold: %f", self._activationThreshold)

    def handle_interferring_correlations(self, behaviour, currently_influenced_sensors):
        """
        Method checks and resolves (if possible) conflicts with other behaviours of a given behaviour
        :param behaviour: the behaviour that is about to be started
        :param currently_influenced_sensors: list of the currently influenced sensors, will be updated if necessary
        :return: tuple(True if this behaviour is interfering and this cannot be resolved, maybe updated currently_influenced_sensors)
        """
        #TODO .get_pddl_effect_name() might not be 100% accurate, reconsider this

        interferingCorrelations = currently_influenced_sensors.intersection(
            set([item.get_pddl_effect_name() for item in behaviour.correlations]))
        # it must not conflict with an already running behaviour ...
        if len(interferingCorrelations) > 0 and not behaviour.manualStart:
            alreadyRunningBehavioursRelatedToConflict = filter(
                lambda x: len(interferingCorrelations.intersection(set([item.get_pddl_effect_name() for item in x.correlations]))) > 0,
                self.__executedBehaviours)  # but it might be the case that the conflicting running behaviour(s) has/have less priority ...
            assert len(
                alreadyRunningBehavioursRelatedToConflict) <= 1  # This is true as long as there are no Aggregators (otherwise those behaviours must have been in conflict with each other).
            # This implementation deals with a list although it it clear that there is at most one element.
            stoppableBehaviours = filter(
                lambda x: x.priority <= behaviour.priority and x.interruptable and not x.manualStart,
                alreadyRunningBehavioursRelatedToConflict)  # only if the behaviour has less priority and is interruptable it should be stopped. Manually started behaviours also cannot be stopped
            # only continue if we can stop ALL offending behaviours. Otherwise we would kill some of them but that
            # doesn't solve the problem and they died for nothing.
            if set(stoppableBehaviours) == set(alreadyRunningBehavioursRelatedToConflict):
                rhbplog.loginfo("%s has conflicting correlations with behaviours %s (%s) that can be solved",
                              behaviour.name, alreadyRunningBehavioursRelatedToConflict, interferingCorrelations)
                for conflictor in stoppableBehaviours: # stop another behaviour in order to resolve the conflict
                    rhbplog.loginfo("STOP BEHAVIOUR %s because it is interruptable and has less priority than %s",
                                  behaviour.name, conflictor.name)
                    self._stop_behaviour(conflictor, True)
                    # stopping a behaviour here requires to recalulate the currently_influenced_sensors
                    currently_influenced_sensors = self._get_currently_influenced_sensors()
                    ### we have now made room for the higher-priority behaviour ###
            else:
                rhbplog.loginfo(
                    "%s will not be started because it has conflicting correlations with already running behaviour(s) %s that cannot be solved (%s)",
                    behaviour.name, alreadyRunningBehavioursRelatedToConflict, interferingCorrelations)
                return True, currently_influenced_sensors
        return False, currently_influenced_sensors

    def _get_currently_influenced_sensors(self):
        """
        Get a set of the currently influenced sensor of all executed behaviours
        :return: set of sensors
        """
        # TODO .get_pddl_effect_name() might not be 100% accurate, reconsider this
        currently_influenced_sensors = set(list(
            itertools.chain.from_iterable([[item.get_pddl_effect_name() for item in x.correlations] for x in self.__executedBehaviours])))
        rhbplog.loginfo("currently influenced sensors: %s", currently_influenced_sensors)
        return currently_influenced_sensors

    def _update_bias_parameters(self):
        """
        Check and update bias parameter values
        """
        conflictor_bias = self.__conflictor_bias if self.__conflictor_bias else rospy.get_param("~conflictorBias", 1.0)
        goal_bias = self.__goal_bias if self.__goal_bias else rospy.get_param("~goalBias", 1.0)
        predecessor_bias = self.__predecessor_bias if self.__predecessor_bias else rospy.get_param("~predecessorBias",
                                                                                                   1.0)
        successor_bias = self.__successor_bias if self.__successor_bias else rospy.get_param("~successorBias", 1.0)
        plan_bias = self.__plan_bias if self.__plan_bias else rospy.get_param("~planBias", 1.0)
        situation_bias = self.__situation_bias if self.__situation_bias else rospy.get_param("~situationBias", 1.0)
        activation_decay = self.__activationDecay if self.__activationDecay else rospy.get_param("~activationDecay", 0.9)
        self.activation_algorithm.update_config(situation_bias=situation_bias, plan_bias=plan_bias,
                                                conflictor_bias=conflictor_bias, goal_bias=goal_bias,
                                                successor_bias=successor_bias, predecessor_bias=predecessor_bias,
                                                activation_decay=activation_decay)

    def _stop_behaviour(self, behaviour, reset_activation=True):
        """
        stop the execution of a behaviour
        :param behaviour: the behaviour to stop
        :param reset_activation: true or false if the activation of the behaviour should be reseted
        """
        behaviour.stop(reset_activation)
        try:
            self.__executedBehaviours.remove(behaviour)  # remove it from the list of executed behaviours
        except ValueError as e:
            rhbplog.logwarn("Tried to stop already stopped behaviour %s", behaviour.name)
        rhbplog.logdebug("Stopped %s still running behaviours: %s", behaviour.name, self.__executedBehaviours)

    def add_goal(self,goal):
        '''
        :param goal: The new goal
        :type goal: AbstractGoalRepresentation
        '''
        with self._step_lock:
            self._goals = filter(lambda x: x.name != goal.name, self._goals) # kick out existing goals with the same name.
            self._goals.append(goal)
            rhbplog.loginfo("A goal with name %s registered", goal.name)
            self.__replanningNeeded = True

    def __add_goal_callback(self, request):
        """
        Callback handler of the AddGoal service
        :param request: ervice request
        :type request: AddGoal
        :return: AddGoalResponse
        """
        goal = GoalProxy(name=request.name, permanent=request.permanent, planner_prefix=self._prefix)
        self.add_goal(goal)
        return AddGoalResponse()

    def add_behaviour(self, behaviour):
        """
        Adds a behaviour to the manager
        :param behaviour: the new behaviour object
        :type behaviour: Behaviour
        """
        with self._step_lock:
            self._behaviours = filter(lambda x: x.name != behaviour.name, self._behaviours) # kick out existing behaviours with the same name.

            behaviour.manager = self
            self._behaviours.append(behaviour)
            rhbplog.loginfo("A behaviour with name %s registered(steps=%r)", behaviour.name,behaviour.requires_execution_steps)
            self.__replanningNeeded = True

    def remove_goal(self, goal_name):
        with self._step_lock:
            self._goals = [g for g in self._goals if
                                g.name != goal_name]  # kick out existing goals with that name.
            self.__replanningNeeded = True

    def remove_behaviour(self, behaviour_name):
        with self._step_lock:
            # kick out existing behaviours with the same name.
            self._behaviours = [b for b in self._behaviours if b.name != behaviour_name]
            self.__replanningNeeded = True

    def __add_behaviour_callback(self, request):
        """
        Callback handler of the AddBehaviour service
        :param request: service request
        :type request: AddBehaviour
        """
        behaviour = Behaviour(name = request.name, planner_prefix=self._prefix, independentFromPlanner = request.independentFromPlanner,
                              requires_execution_steps = request.requiresExecutionSteps,
                              create_log_files = self._create_log_files, log_file_path_prefix=self.__log_file_path_prefix)
        self.add_behaviour(behaviour=behaviour)
        return AddBehaviourResponse()
    
    def __pause_callback(self, dummy):
        with self._step_lock: #ensures that the manager is really not just doing something and the next step is blocked
            self.pause_counter += 1
            rhbplog.logdebug('Manager Paused')
            return EmptyResponse()
    
    def __resume_callback(self, dummy):
        with self._step_lock: #ensures that the manager is really not just doing something and the next step is blocked
            if self.pause_counter > 0:
                self.pause_counter -= 1
                rhbplog.logdebug('Manager Resumed')
            return EmptyResponse()
    
    def __remove_goal_callback(self, request):
        self.remove_goal(goal_name=request.name)
        return RemoveGoalResponse()

    def __remove_behaviour_callback(self, request):
        self.remove_behaviour(behaviour_name=request.name)
        return RemoveBehaviourResponse()
    
    def __manual_start_callback(self, request):
        for behaviour in self._behaviours:
            if behaviour.name == request.name:
                behaviour.manualStart = request.forceStart
                break
        return ForceStartResponse()
    
    @property
    def activationThreshold(self):
        return self._activationThreshold
    
    @activationThreshold.setter
    def activationThreshold(self, threshold):
        self._activationThreshold = threshold

    @property
    def goals(self):
        return self._goals
    
    @property
    def behaviours(self):
        return self._behaviours
    
    @property
    def activeBehaviours(self):
        return self._activeBehaviours
    
    @property
    def activeGoals(self):
        return self._activeGoals
    
    @property
    def totalActivation(self):
        return self._totalActivation
    
    @property
    def planExecutionIndex(self):
        return self._planExecutionIndex

    @property
    def activated(self):
        return self.__activated
    
    @property
    def plan(self):
        if self._plan:
            return self._plan
        else:
            return None

    def deactivate(self):
        self.__activated = False
        #use while to avoid illegal state of non running behaviors in __executedBehaviors
        while (len(self.__executedBehaviours)>0):
            behaviour = self.__executedBehaviours[0]
            self.__executedBehaviours.remove(behaviour)  # remove it from the list of executed behaviours
            behaviour.stop(True)
        for behaviour in self._behaviours:
            behaviour.reset_activation()

    def activate(self):
        self.__activated = True

    def is_interruptible(self):
        if (self.__use_only_running_behaviors_for_interruptible):
            relevant_behaviors = self.__executedBehaviours
        else:
            relevant_behaviors = self._behaviours
        for behavior in relevant_behaviors:
            if (not behavior.interruptable):
                return False
        return True


class ManagerControl(object):
    '''
    Helper class for remotely controlling the manager
    '''

    def __init__(self, plannerPrefix=""):
        self.__plannerPrefix = plannerPrefix

    def pause(self):
        service_name = self.__plannerPrefix + '/' + 'Pause'
        rhbplog.logdebug("Waiting for service %s", service_name)
        rospy.wait_for_service(service_name)
        pauseRequest = rospy.ServiceProxy(service_name, Empty)
        pauseRequest()

    def resume(self):
        service_name = self.__plannerPrefix + '/' + 'Resume'
        rhbplog.logdebug("Waiting for service %s", service_name)
        rospy.wait_for_service(service_name)
        resumeRequest = rospy.ServiceProxy(service_name, Empty)
        resumeRequest()
