"""
Created on 23.04.2015

@author: wypler, hrabia
"""

import sys
import threading

import rospy
import itertools

from std_srvs.srv import Empty, EmptyResponse
from rhbp_core.msg import PlannerStatus, Status, DiscoverInfo
from rhbp_core.srv import AddBehaviour, AddBehaviourResponse, AddGoal, AddGoalResponse, RemoveBehaviour, \
    RemoveBehaviourResponse, RemoveGoal, RemoveGoalResponse, ForceStart, ForceStartResponse, GetPaused, \
    GetPausedResponse, PlanWithGoal, PlanWithGoalResponse, SetStepping
from rospy import ROSInterruptException
from .behaviours import Behaviour
from .goals import GoalProxy
from .pddl import PDDL, mergeStatePDDL, tokenizePDDL, getStatePDDLchanges, predicateRegex, init_missing_functions, \
    create_valid_pddl_name, aggregate_sensor_changes, parseStatePDDL
from .planner import MetricFF
from .activation_algorithm import ActivationAlgorithmFactory
from utils.misc import LogFileWriter
from copy import copy

from dynamic_reconfigure.server import Server
from dynamic_reconfigure.msg import Config as ConfigMsg
from dynamic_reconfigure import encoding
from rhbp_core.cfg import ManagerConfig

import utils.rhbp_logging

rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.planning')


class Manager(object):
    USE_ONLY_RUNNING_BEHAVIOURS_FOR_INTERRUPTIBLE_DEFAULT_VALUE = False

    MANAGER_DISCOVERY_TOPIC = "rhbp_discover"

    dynamic_reconfigure_server = None

    '''
    This is the manager class that keeps track of all elements in the network (behaviours, goals, sensors).
    Behaviours need this to know what sensors exist in the world and how they are correlated their measurement.
    They also need to know the goals to decide whether they are supporting them (which increases their activation) or inhibiting them (which decreases their activation).
    And last, they need to know whether another behaviour conflicts with them to reduce the activation of it.
    Also global constants like activation thresholds are stored here.
    '''

    def __init__(self, enabled=True,
                 use_only_running_behaviors_for_interRuptible=USE_ONLY_RUNNING_BEHAVIOURS_FOR_INTERRUPTIBLE_DEFAULT_VALUE,
                 **kwargs):
        '''
        Constructor
        '''
        self._prefix = kwargs["prefix"] if "prefix" in kwargs else "" # if you have multiple planners in the same ROS environment use this to distinguish between the instances
        self._param_prefix = self._prefix + '/rhbp_manager'

        self._sensors = [] #TODO this is actually not used at all in the moment, only behaviour know the sensors and activators
        self._goals = []
        # pre-computed (in step()) list of not yet fulfilled goals that are working fine
        self._operational_goals = []
        self._behaviours = []
        # pre-computed (in step()) list of operational behaviours that are working fine
        self._operational_behaviours = []
        self._totalActivation = 0.0  # pre-computed (in step()) sum all activations of operational behaviours
        self._activationThreshold = kwargs["activationThreshold"] if "activationThreshold" in kwargs \
            else rospy.get_param(self._param_prefix + "/activationThreshold", 7.0)  # not sure how to set this just yet.

        # decay for the activation threshold per step, configurable with dynamic configure
        self._activation_threshold_decay = 0.9

        self._create_log_files = kwargs["createLogFiles"] if "createLogFiles" in kwargs else rospy.get_param(
            self._param_prefix + "/createLogFiles", False)

        # configures if all contained behaviour or only the executed behaviours are used to determine if the manager is
        # interruptable
        self.__use_only_running_behaviors_for_interruptible = use_only_running_behaviors_for_interRuptible

        self.__max_parallel_behaviours = kwargs['max_parallel_behaviours'] if 'max_parallel_behaviours' in kwargs else \
            rospy.get_param(self._param_prefix + "/max_parallel_behaviours", sys.maxint)

        rhbplog.loginfo("Using max_parallel_behaviours:%d", self.__max_parallel_behaviours)

        self._stepCounter = 0

        self._step_lock = threading.Lock()

        self.__log_file_path_prefix = self._prefix + '/' if self._prefix else ''

        if self._create_log_files:
            self.__threshFile = LogFileWriter(path=self.__log_file_path_prefix, filename="threshold", extension=".log")
            self.__threshFile.write("{0}\t{1}\n".format("Time", "activationThreshold"))
            rhbplog.loginfo("Writing additional logfiles to: %s", self.__log_file_path_prefix)
        else:
            rhbplog.loginfo("Writing additional logfiles is disabled.")

        self.__replanningNeeded = False  # this is set when behaviours or goals are added or removed, or the last planning attempt returned an error.
        self.__previousStatePDDL = PDDL()
        self.__previous_parsed_state_pddl = {}
        self.__sensorChanges = {} # this dictionary stores all sensor changes between two steps in the form {sensor name <string> : indicator <float>}. Note: the float is not scaled [-1.0 to 1.0] but shows a direction (positive or negative).
        self.__aggregated_sensor_changes = {} # This dictionary stores the accumlated changes for one step of the PDDL planner

        # Flags for the properties that are considered for triggering replanning, they are configurable via
        # dynamic reconfigure
        self._plan_monitoring_all_sensor_changes_by_behaviours = True
        self._plan_monitoring_behaviour_missing_influence = True
        self._plan_monitoring_unexpected_behaviour_finished = True

        self._plan = {}
        self._planExecutionIndex = 0
        self.__goalPDDLs = {}
        self.__last_domain_PDDL = ""
        self._currently_pursued_goals = []

        self.planner = MetricFF()

        # create activation algorithm
        algorithm_name = kwargs['activation_algorithm'] if 'activation_algorithm' in kwargs else \
            rospy.get_param(self._param_prefix + "/activation_algorithm", 'default')

        rhbplog.loginfo("Using activation algorithm: %s", algorithm_name)
        self.activation_algorithm = ActivationAlgorithmFactory.create_algorithm(algorithm_name, self)

        self.pause_counter = 0  # counts pause requests, step is only executed at pause_counter = 0
        self.__enable = enabled

        self.init_services_topics()

        self.__executedBehaviours = []

        rhbplog.loginfo("RHBP manager with prefix '%s' is started.", self._prefix)

    def init_services_topics(self):
        self._service_prefix = self._prefix + '/'
        self.__addBehaviourService = rospy.Service(self._service_prefix + 'AddBehaviour', AddBehaviour,
                                                   self.__add_behaviour_callback)
        self.__addGoalService = rospy.Service(self._service_prefix + 'AddGoal', AddGoal, self.__add_goal_callback)
        self.__removeBehaviourService = rospy.Service(self._service_prefix + 'RemoveBehaviour', RemoveBehaviour,
                                                      self.__remove_behaviour_callback)
        self.__removeGoalService = rospy.Service(self._service_prefix + 'RemoveGoal', RemoveGoal,
                                                 self.__remove_goal_callback)
        self.__manualStartService = rospy.Service(self._service_prefix + 'ForceStart', ForceStart,
                                                  self.__manual_start_callback)
        self.__pauseService = rospy.Service(self._service_prefix + 'Pause', Empty, self.__pause_callback)
        self.__resumeService = rospy.Service(self._service_prefix + 'Resume', Empty, self.__resume_callback)
        self.__get_paused_service = rospy.Service(self._service_prefix + 'GetPaused', GetPaused, self.__get_paused_callback)
        self.__plan_with_goal = rospy.Service(self._service_prefix + 'PlanWithGoal', PlanWithGoal,
                                              self.__plan_with_registered_goals_callback)
        self.__statusPublisher = rospy.Publisher(self._service_prefix + 'Planner/plannerStatus', PlannerStatus,
                                                 queue_size=1, latch=True)

        self.__pub_discover = rospy.Publisher(name=Manager.MANAGER_DISCOVERY_TOPIC, data_class=DiscoverInfo,
                                              queue_size=1)

        if not Manager.dynamic_reconfigure_server:  # only one server per node
            Manager.dynamic_reconfigure_server = Server(ManagerConfig, self._dynamic_reconfigure_callback,
                                                        namespace="/" + self._param_prefix)
        else:
            self.__config_subscriber = rospy.Subscriber(Manager.dynamic_reconfigure_server.ns + 'parameter_updates',
                                                        ConfigMsg, self._dynamic_reconfigure_listener_callback)

    def unregister(self):
        """
        Unregister all remote services of the manager, this should only be called when the manager instance is not used
        any longer
        """
        self.__addBehaviourService.shutdown()
        self.__addGoalService.shutdown()
        self.__removeBehaviourService.shutdown()
        self.__removeGoalService.shutdown()
        self.__manualStartService.shutdown()
        self.__pauseService.shutdown()
        self.__get_paused_service.shutdown()
        self.__resumeService.shutdown()
        self.__plan_with_goal.shutdown()
        self.__statusPublisher.unregister()
        self.__pub_discover.unregister()

    def __del__(self):
        self.unregister()

    def reset(self):
        """
        Reset internal decision_state by stopping running behaviours, reseting thresholds etc
        """
        for b in self.executed_behaviours:
            self._stop_behaviour(b)
        self.__executedBehaviours = []

        self._plan = {}
        self._planExecutionIndex = 0
        self.__goalPDDLs = {}
        self.__last_domain_PDDL = ""
        self._currently_pursued_goals = []
        self.__replanningNeeded = True
        self._totalActivation = 0.0  # pre-computed (in step()) sum all activations of operational behaviours
        self._activationThreshold = rospy.get_param(self._param_prefix + "/activationThreshold", 7.0)

    def _getDomainName(self):
        return create_valid_pddl_name(self._prefix) if self._prefix else "UNNAMED"

    def _fetchPDDL(self, behaviours, goals):
        '''
        This method fetches the PDDL from all behaviours and goals, merges the state descriptions and returns the domain
        PDDL string ready for ff. Alongside it updates
        As a side effect, is also computes the sensor changes that happened between the last invocation and now.
        '''
        behaviourPDDLs = [behaviour.fetchPDDL() for behaviour in behaviours]
        self.__goalPDDLs = {goal: goal.fetchPDDL() for goal in goals}
        pddl = PDDL()
        # Get relevant domain information from behaviour pddls
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
        # Update requirements if necessary
        # Actually :fluents could just be :numeric-fluents, but this is not accepted by metric-ff
        domainPDDLString += "(:requirements :strips :adl :equality :negation :conditional-effects :fluents)\n"
        domainPDDLString += "(:predicates\n    " + "\n    ".join("({0})".format(x) for x in pddl.predicates) + ")\n"
        domainPDDLString += "(:functions\n    " + "\n    ".join("({0})".format(x) for x in pddl.functions) + ")\n"
        domainPDDLString += pddl.statement + ")"
        merged_state_pddl = PDDL()
        for _actionPDDL, statePDDL in behaviourPDDLs:
            merged_state_pddl = mergeStatePDDL(statePDDL, merged_state_pddl)
        for v in self.__goalPDDLs.itervalues():
            if not v is None:
                _goalPDDL, statePDDL = v
                merged_state_pddl = mergeStatePDDL(statePDDL, merged_state_pddl)

        merged_state_pddl = init_missing_functions(pddl, merged_state_pddl)

        # we parse the PDDL before negative predicates are filtered out
        new_parsed_state_pddl = parseStatePDDL(merged_state_pddl)

        # filter out negative predicates. FF can't handle them!
        statePDDL = PDDL(statement="\n\t\t".join(
            filter(lambda x: predicateRegex.match(x) is None or predicateRegex.match(x).group(2) is None, tokenizePDDL(merged_state_pddl.statement)))
        )# if the regex does not match it is a function (which is ok) and if the second group is None it is not negated (which is also ok)

        # compute changes
        new_sensor_changes = getStatePDDLchanges(self.__previous_parsed_state_pddl, new_parsed_state_pddl)
        self.__aggregated_sensor_changes = aggregate_sensor_changes(old_changes=self.__sensorChanges,
                                                                    new_changes=new_sensor_changes)
        self.__sensorChanges = new_sensor_changes
        self.__previousStatePDDL = statePDDL
        self.__previous_parsed_state_pddl = new_parsed_state_pddl
        self.__last_domain_PDDL = domainPDDLString
        return domainPDDLString

    def _reset_sensor_changes(self):
        self.__aggregated_sensor_changes = {}

    def _create_problem_pddl(self, goals):
        '''
        This method creates the problem PDDL for a given set of goals.
        It relies on the fact that self.fetchPDDL() has run before and filled the self.__goalPDDLs dictionary with the most recent responses from the actual goals and self.__previousStatePDDL with the CURRENT state PDDL
        '''
        goalConditions = (self.__goalPDDLs[goal][0].statement for goal in goals) # self.__goalPDDLs[goal][0] is the goalPDDL of goal's (goalPDDL, statePDDL) tuple
        problemPDDLString = self._create_problem_pddl_string(" ".join(goalConditions))

        return problemPDDLString

    def _create_problem_pddl_string(self, goal_conditions_string):
        """
        Creates a problem PDDL-String for a given String, that contains all goal statements

        :param goal_conditions_string: string containing all goal statements separated by a space
        :return: problemPDDLString
        """

        problemPDDLString = "(define (problem problem-{0})\n\t(:domain {0})\n\t(:init \n\t\t{1}\n\t)\n".format(
            self._getDomainName(), self.__previousStatePDDL.statement)  # at this point the "previous" is the current state PDDL
        problemPDDLString += "\t(:goal (and {0}))\n\t(:metric minimize (costs))\n".format(goal_conditions_string)
        problemPDDLString += ")\n"
        return problemPDDLString

    def _log_pddl_files(self, domainPDDLString, problemPDDLString, goals):

        filename = "pddl{0}Domain".format(self._stepCounter)
        domainLog = LogFileWriter(path=self.__log_file_path_prefix, filename=filename, extension=".pddl")
        domainLog.write(domainPDDLString)

        filename = "pddl{0}Problem_{1}".format(self._stepCounter, ''.join((str(g) for g in goals)))
        problemLog = LogFileWriter(path=self.__log_file_path_prefix, filename=filename, extension=".pddl")
        problemLog.write(problemPDDLString)

    def _generate_priority_goal_sequences(self):
        '''
        This is a generator that generates goal sequences with descending priorities.
        It yields sorted lists with the most important goal at the front and strips away one element from the back at each iteration.
        After the most important goal was the only remaining element in the list the same process repeats for the second most important goals and so on.
        '''
        sortedGoals = sorted(self._operational_goals, key=lambda x: x.priority, reverse=True)
        numElements = len(sortedGoals)
        for i in xrange(0, numElements, 1):
            for j in xrange(numElements, i, -1):
                yield sortedGoals[i:j]
    
    def _plan_if_necessary(self):
        """
        this method plans using the symbolic planner if it is required to do so.
        Replanning is required whenever any or many of the following conditions is/are met:
        1) Behaviours or Goals have been added or removed
        2) An interruptable behaviour timed out
        3) The last planning attempt was unsuccessful

        Additionally, following properties can also initiate replanning if their flags are enabled. Configuration is
        possible with dynamic reconfigure respectively from the launch file.
        1) The state of the world has changed an this change is not caused by the current behaviour in the plan
            Flag: self._plan_monitoring_all_sensor_changes_by_behaviours.
        2) The expected effect (direction of change) of executed behaviours did not take place
            Flag: self._plan_monitoring_behaviour_missing_influence
        3) An unexpected behaviour was running (not head of the plan or flagged as independentFromPlanner)
            Flag: self._plan_monitoring_unexpected_behaviour_finished
        """

        # return directly if planner is disabled or we are missing goals or behaviours
        if not self.activation_algorithm.is_planner_enabled() \
                or len(self._operational_behaviours) == 0 or len(self._operational_goals) == 0:
            return

        # _fetchPDDL also updates our self.__sensorChanges and self.__goalPDDLs dictionaries
        domain_pddl = self._fetchPDDL(behaviours=self._operational_behaviours, goals=self._operational_goals)

        rhbplog.logdebug("Changes: %s", self.__sensorChanges)

        planned_behaviour_effects_realised = self._are_effects_of_planned_behaviour_realised()

        if planned_behaviour_effects_realised:
            self._planExecutionIndex += 1
            self._reset_sensor_changes()

        if self._plan_monitoring_all_sensor_changes_by_behaviours:
            if not self.__replanningNeeded:  # we do not have to test this if replanning is required anyhow
                all_changes_were_not_expected = not self._are_all_sensor_changes_from_executed_behaviours()
            else:
                all_changes_were_not_expected = None
        else:
            all_changes_were_not_expected = False

        if self._plan_monitoring_behaviour_missing_influence:
            if not self.__replanningNeeded:  # we do not have to test this if replanning is required anyhow
                executed_behaviours_missing_effect_influence = not self._executed_behaviours_influenced_as_expected()
            else:
                executed_behaviours_missing_effect_influence = None
        else:
            executed_behaviours_missing_effect_influence = False

        # here we have to do the processing to handle the plan index increment
        unexpected_behaviour_finished = self._finished_unexpected_behaviour(
            increment_planning_step=not planned_behaviour_effects_realised)  # don't increment if successfully realised

        if not self._plan_monitoring_unexpected_behaviour_finished:
            unexpected_behaviour_finished = False

        # now, we know whether we need to plan again or not
        if self.__replanningNeeded or unexpected_behaviour_finished or all_changes_were_not_expected \
            or executed_behaviours_missing_effect_influence:
            rhbplog.logdebug("### PLANNING ### because replanning needed: %s\n"
                            "planIndex: %s, unexpected_behaviour_finished:%s, all_changes_were_not_expected:%s, "
                            "planned_behaviour_effects_realised: %s, executed_behaviours_missing_influence:%s",
                            self.__replanningNeeded, self._planExecutionIndex, unexpected_behaviour_finished,
                            all_changes_were_not_expected, planned_behaviour_effects_realised,
                            executed_behaviours_missing_effect_influence)

            # now we need to make the best of our planning problem (Trying to fulfill as many goals as possible):
            # In cases where the full set of goals can't be reached because the planner does not find a solution a
            # reduced set is used.
            # The reduction will eliminate goals of inferior priority until the highest priority goal is tried alone.
            # If that cannot be reached the search goes backwards and tries all other goals with lower priorities in
            # descending order until a reachable goal is found.
            for goal_sequence in self._generate_priority_goal_sequences():
                problem_pddl = ""
                try:
                    rhbplog.logdebug("trying to reach goals %s", goal_sequence)
                    problem_pddl = self._create_problem_pddl(goal_sequence)

                    tmp_plan = self.planner.plan(domain_pddl, problem_pddl)
                    if tmp_plan and "cost" in tmp_plan and tmp_plan["cost"] != -1.0:
                        rhbplog.loginfo("FOUND PLAN: %s", tmp_plan)
                        self._plan = tmp_plan
                        self.__replanningNeeded = False
                        self._planExecutionIndex = 0
                        self._reset_sensor_changes()
                        self._currently_pursued_goals = goal_sequence
                        break
                    else:
                        rhbplog.loginfo("PROBLEM IMPOSSIBLE")
                        # resetting the plan to avoid that we try to follow an impossible plan.
                        self._planExecutionIndex = 0
                        self._plan = {}
                    if self._create_log_files:
                        self._log_pddl_files(domain_pddl, problem_pddl, goal_sequence)
                except Exception as e:
                    rhbplog.logerr("PLANNER ERROR: %s. Generating PDDL log files for step %d", e, self._stepCounter)
                    self.__replanningNeeded = True  # in case of planning exceptions try again next iteration
                    self._planExecutionIndex = 0
                    self._plan = {}
                    self._log_pddl_files(domain_pddl, problem_pddl, goal_sequence)
        else:
            rhbplog.loginfo("### NOT PLANNING ### because replanning needed: %s\n"
                            "planIndex: %s, unexpected_behaviour_finished:%s, all_changes_were_not_expected:%s, "
                            "planned_behaviour_effects_realised: %s, executed_behaviours_missing_influence:%s",
                            self.__replanningNeeded, self._planExecutionIndex, unexpected_behaviour_finished,
                            all_changes_were_not_expected, planned_behaviour_effects_realised,
                            executed_behaviours_missing_effect_influence)

    def _finished_unexpected_behaviour(self, increment_planning_step):
        """
        # The method tracks progress on the plan and finds out if an unexpected behaviour finished.
        # plus if an expected behaviour finished the self._planExecutionIndex will be incremented if the parameter
        increment_planning_step==True
        :param increment_planning_step: set to True if the self._planExecutionIndex should be incremented
        :return: True if an unexpected behaviour finished
        """
        unexpected_behaviour_finished = False
        # make sure the finished behaviour was part of the plan at all (otherwise it is unexpected)
        if self._plan:
            for behaviour in self.__executedBehaviours:
                if behaviour.justFinished and not behaviour.independentFromPlanner and behaviour.name not in \
                       [name for index, name in self._plan["actions"].iteritems() if index >= self._planExecutionIndex]:
                    unexpected_behaviour_finished = True  # it was unexpected
                    break
            # if we found a behaviour that executed that was not part of the plan (and not flagged independentFromPlan)
            # we can stop here, otherwise we have to ensure that the behaviours finished in correct order.
            if not unexpected_behaviour_finished:
                # walk along the remaining plan
                for index in filter(lambda x: x >= self._planExecutionIndex, sorted(self._plan["actions"].keys())):
                    # only those may be finished. the others were not even running
                    for behaviour in self.__executedBehaviours:
                        # inspect the behaviour at the current index of the plan
                        if behaviour.name == self._plan["actions"][index] and behaviour.justFinished:
                            # if it was a planned behaviour and it finished
                            if increment_planning_step and self._planExecutionIndex == index:
                                self._planExecutionIndex += 1  # we are one step ahead in our plan
                                self._reset_sensor_changes()
                            # otherwise and if the behaviour was not allowed to act reactively on its own
                            # (flagged as independentFromPlanner)
                            elif not behaviour.independentFromPlanner:
                                unexpected_behaviour_finished = True  # it was unexpected
        return unexpected_behaviour_finished

    def _executed_behaviours_influenced_as_expected(self):
        """
        check if all modelled effects of the executed behaviours are reflected in the sensor changes.
        :return: False if one of the modelled effects is not listed in the current sensor changes
        """

        for behaviour in self.__executedBehaviours:
            for item in behaviour.correlations:
                if item.indicator == 0:
                    continue  # does not make much sense to model an effect of 0 but in case.

                sensor_name = item.get_pddl_effect_name()
                sensor_change_indicator = self.__sensorChanges.get(sensor_name, None)
                # none would indicate that we currently not tracking this sensor in any condition and hence do not
                # have information about it
                if not sensor_change_indicator:
                    continue
                elif item.indicator * sensor_change_indicator <= 0:
                    current_state = self.__previous_parsed_state_pddl.get(sensor_name, None)
                    # test if sensor is a boolean predicated that is already in the desired state
                    if current_state and isinstance(current_state, bool):
                        if current_state and item.indicator > 0 or not current_state and item.indicator <= 0:
                            continue
                    return False
        return True

    def _are_all_sensor_changes_from_executed_behaviours(self):
        """
        check if all sensor changes are induced by executed behaviours
        no change is also considered as an expected change
        :return: False if at least one sensor change (direction of change) is not modeled as effect of an executed behaviour
        """
        # now check whether we expected the world to change so by comparing the observed changes to the correlations of the running behaviours
        all_changes_were_expected = True
        for sensor_name, indicator in self.__sensorChanges.iteritems():
            change_was_expected = False

            for behaviour in self.__executedBehaviours:
                for item in behaviour.correlations:
                    # rhbplog.logdebug("Behaviour: %s, Correlations: %s, Indicator: %f", behaviour.name, item.get_pddl_effect_name(), item.indicator)
                    # the observed change happened because of the running behaviour (at least the behaviour is
                    # correlated to the changed sensor in the correct way)
                    if item.get_pddl_effect_name() == sensor_name and item.indicator * indicator > 0:
                        change_was_expected = True
                        break
                if change_was_expected:
                    break
            if not change_was_expected:
                rhbplog.logdebug("Change '%s':%f was not expected", sensor_name, indicator)
                all_changes_were_expected = False
                break
        return all_changes_were_expected

    def _are_effects_of_planned_behaviour_realised(self):
        """
        check if the currently planned behaviour did its job
        :return: True if all expected/modelled effects of the currently planned behaviour have been realised
        """
        effect_realised = False
        if self._plan and "actions" in self._plan and self._planExecutionIndex in self._plan["actions"]:
            planned_name = self._plan["actions"][self._planExecutionIndex]
            planned_executed_behaviour = None
            for behaviour in self.__executedBehaviours:
                if behaviour.name == planned_name:
                    planned_executed_behaviour = behaviour
                    break
            if planned_executed_behaviour:
                for item in planned_executed_behaviour.correlations:
                    sensor_name = item.get_pddl_effect_name()
                    aggregated_sensor_change = self.__aggregated_sensor_changes.get(sensor_name, 0)

                    if item.indicator < aggregated_sensor_change:

                        current_state = self.__previous_parsed_state_pddl.get(sensor_name, None)
                        # test if sensor is a boolean predicated that is already in the desired state
                        if current_state and isinstance(current_state, bool):
                            if current_state and item.indicator > 0 or not current_state and item.indicator <= 0:
                                effect_realised = True
                                continue

                        effect_realised = False
                        break
                    else:
                        effect_realised = True
        return effect_realised

    def send_discovery(self):
        """
        Send manager/planner discovery message
        """
        msg = DiscoverInfo()
        msg.stamp = rospy.Time.now()
        msg.manager_prefix = self._prefix
        self.__pub_discover.publish(msg)

    def step(self, force=False, guarantee_decision=False):
        """
        Execute one decision-making step
        :param force: set to True to force the stepping regardless of pause and activation states
        :param guarantee_decision: Repeat activation algorithm threshold adjustments until at least one behaviour can
                                   be activated and enough executable behaviours are available
        """
        if not force and ((self.pause_counter > 0) or (not self.__enable)):
            return

        if self._create_log_files:  # debugging only
            self.__threshFile.append("{0:f}\t{1:f}\n".format(rospy.get_time(), self._activationThreshold))

        self.send_discovery()

        with self._step_lock:
            rhbplog.logdebug("###################################### STEP {0} ######################################"
                             .format(self._stepCounter))

            self.update_activation()

            while True:  # do while loop for guarantee_decision

                rhbplog.loginfo("############## ACTIONS ###############")
                # actually, operational_behaviours should be enough as search space but if the behaviour implementer
                # resets active before isExecuting we are safe this way
                self.__executedBehaviours = filter(lambda x: x.isExecuting, self._behaviours)
                amount_of_manually_startable_behaviours = len(filter(lambda x: x.manualStart, self._behaviours))
                amount_started_behaviours = 0
                amount_currently_selected_behaviours = 0

                rhbplog.loginfo("currently running behaviours: %s", self.__executedBehaviours)

                currently_influenced_sensors = set()

                # perform the decision making based on the calculated activations
                for behaviour in sorted(self._behaviours, key=lambda x: x.activation, reverse=True):
                    ### now comes a series of tests that a behaviour must pass in order to get started ###
                    if not behaviour.active and not behaviour.manualStart:  # it must be active
                        rhbplog.loginfo("'%s' will not be started because it is not active", behaviour.name)
                        continue
                    # Behaviour is not already running and is not manually activated
                    if behaviour.isExecuting:
                        # It is important to remember that only interruptable behaviours can be stopped by the manager
                        if behaviour.executionTimeout != -1 and behaviour.executionTime >= behaviour.executionTimeout \
                                and behaviour.interruptable and not behaviour.manualStart:
                            rhbplog.loginfo("STOP BEHAVIOUR '%s' because it timed out", behaviour.name)
                            self._stop_behaviour(behaviour, True)
                            self.__replanningNeeded = True  # this is unusual so we trigger replanning
                        elif not behaviour.executable and behaviour.interruptable and not behaviour.manualStart:
                            rhbplog.loginfo("STOP BEHAVIOUR '%s' because it is not executable anymore", behaviour.name)
                            self._stop_behaviour(behaviour, True)
                        elif behaviour.activation < self._activationThreshold and behaviour.interruptable \
                                and not behaviour.manualStart:
                            rhbplog.loginfo("STOP BEHAVIOUR '%s' because of too low activation %f < %f",
                                            behaviour.name, behaviour.activation, self._activationThreshold)
                            self._stop_behaviour(behaviour, False)
                        elif self.__max_parallel_behaviours > 0 and behaviour.interruptable and \
                                amount_currently_selected_behaviours >= self.__max_parallel_behaviours \
                                and not behaviour.manualStart:
                            # if we try to stop non-interruptable behaviours and we have already to many behaviours running
                            # this should be resolved in the next decision-making round
                            rhbplog.loginfo("STOP BEHAVIOUR '%s' because of too many executed behaviours", behaviour.name)
                            self._stop_behaviour(behaviour, False)
                        else:
                            rhbplog.logdebug("'%s' will not be started because it is already executing", behaviour.name)
                            behaviour.executionTime += 1

                            if behaviour.manualStart:
                                amount_of_manually_startable_behaviours -= 1

                            amount_currently_selected_behaviours += 1

                            if behaviour.requires_execution_steps:
                                behaviour.do_step()
                        continue
                    if not behaviour.executable and not behaviour.manualStart:  # it must be executable
                        rhbplog.loginfo("%s will not be started because it is not executable", behaviour.name)
                        continue
                    if behaviour.activation < self._activationThreshold and not behaviour.manualStart:  # it must have high-enough activation
                        rhbplog.loginfo("%s will not be started because it has not enough activation (%f < %f)", behaviour.name, behaviour.activation, self._activationThreshold)
                        continue

                    currently_influenced_sensors = self._get_currently_influenced_sensors()

                    behaviour_is_interfering_others, currently_influenced_sensors, stopped_behaviour_amount = \
                        self.handle_interfering_correlations(behaviour, currently_influenced_sensors)

                    amount_currently_selected_behaviours -= stopped_behaviour_amount  # address resolved conflicts

                    if behaviour_is_interfering_others:
                        continue

                    if behaviour.manualStart:
                        rhbplog.loginfo("BEHAVIOUR %s IS STARTED BECAUSE OF MANUAL REQUEST", behaviour.name)
                    # Do not execute more behaviours than allowed, behaviours are ordered by descending activation hence
                    # we prefer behaviours with higher activation
                    elif self.__max_parallel_behaviours > 0 and \
                                    (amount_currently_selected_behaviours + amount_of_manually_startable_behaviours) \
                                    >= self.__max_parallel_behaviours:
                        rhbplog.logdebug("BEHAVIOUR %s IS NOT STARTED BECAUSE OF TOO MANY PARALLEL BEHAVIOURS", behaviour.name)
                        continue

                    ### if the behaviour got here it really is ready to be started ###
                    rhbplog.loginfo("STARTING BEHAVIOUR %s", behaviour.name)
                    behaviour.start()                                

                    amount_currently_selected_behaviours += 1

                    self.__executedBehaviours.append(behaviour)
                    amount_started_behaviours += 1
                    rhbplog.loginfo("now running behaviours: %s", self.__executedBehaviours)

                # Reduce or increase the activation threshold based on executed and started behaviours
                if len(self.__executedBehaviours) == 0 and len(self._operational_behaviours) > 0:
                    self._activationThreshold *= self._activation_threshold_decay
                    rhbplog.loginfo("REDUCING ACTIVATION THRESHOLD TO %f", self._activationThreshold)
                elif amount_started_behaviours > 0:
                    rhbplog.loginfo("INCREASING ACTIVATION THRESHOLD TO %f", self._activationThreshold)
                    self._activationThreshold *= (1 / self._activation_threshold_decay)

                executable_behaviours = [b for b in self._behaviours if b.executable and b.activation > 0]
                if not guarantee_decision or len(self.__executedBehaviours) > 0 or len(executable_behaviours) == 0:
                    # at least one behaviour is executing or there is no executable behaviour available
                    break
                else:
                    rhbplog.loginfo("No decision_taken repeating with adjusted threshold. New activation threshold: %f",
                                    self._activationThreshold)

            self._publish_planner_status(currently_influenced_sensors)

        self._stepCounter += 1

    def _publish_planner_status(self, currently_influenced_sensors):
        """
        Collect all information for the plannerStatusMessage and publish it
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
            statusMessage.enabled = goal.enabled
            statusMessage.satisfaction = goal.fulfillment
            statusMessage.priority = goal.priority
            plannerStatusMessage.goals.append(statusMessage)

        # collect all that stuff for the rqt gui
        for behaviour in self._behaviours:
            statusMessage = Status()
            statusMessage.name = behaviour.name
            statusMessage.activation = behaviour.activation
            statusMessage.activations = behaviour.activation_components
            statusMessage.satisfaction = behaviour.preconditionSatisfaction
            statusMessage.isExecuting = behaviour.isExecuting
            statusMessage.executionTimeout = behaviour.executionTimeout
            statusMessage.executionTime = behaviour.executionTime
            statusMessage.executable = behaviour.executable
            statusMessage.threshold = behaviour.readyThreshold
            statusMessage.priority = behaviour.priority
            statusMessage.interruptable = behaviour.interruptable
            statusMessage.independentFromPlanner = behaviour.independentFromPlanner
            statusMessage.enabled = behaviour.enabled
            statusMessage.active = behaviour.active
            statusMessage.correlations = [correlation.get_msg() for correlation in behaviour.correlations]
            statusMessage.wishes = [w.get_wish_msg() for w in behaviour.wishes]
            plannerStatusMessage.behaviours.append(statusMessage)
        plannerStatusMessage.runningBehaviours = [b.name for b in self.__executedBehaviours]
        plannerStatusMessage.influencedSensors = list(currently_influenced_sensors)
        plannerStatusMessage.activationThresholdDecay = self._activation_threshold_decay
        plannerStatusMessage.stepCounter = self._stepCounter
        if self._plan and "actions" in self._plan:
            plannerStatusMessage.plan = self._plan['actions'].values()
        plannerStatusMessage.plan_index = self._planExecutionIndex
        self.__statusPublisher.publish(plannerStatusMessage)

    def update_activation(self, plan_if_necessary=True):
        """
        Update all information about behaviours and goals and update the activation calculation
        :param plan_if_necessary: enable or disable potentially required planning
        """
        self._totalActivation = 0.0
        ### collect information about behaviours ###
        for behaviour in self._behaviours:
            behaviour.fetchStatus(self._stepCounter)
            if behaviour.operational:
                self._totalActivation += behaviour.activation
        if self._totalActivation == 0.0:
            self._totalActivation = 1.0  # the behaviours are going to divide by this value so make sure it is non-zero
        rhbplog.logdebug("############# GOAL STATES #############")
        ### collect information about goals ###
        for goal in self._goals:
            goal.fetchStatus(self._stepCounter)
            rhbplog.logdebug("%s: enabled: %s, operational: %s, fulfillment: %f, wishes %s", goal.name, goal.enabled,
                             goal.operational, goal.fulfillment, goal.wishes)
            # Deactivate non-permanent and satisfied goals
            if goal.enabled and not goal.isPermanent and goal.satisfied:
                goal.enabled = False
                rhbplog.logdebug("Set 'enabled' of %s goal to False", goal.name)
        ### do housekeeping ###
        # operational goals and behaviours have to be determined BEFORE computeActivation() of the behaviours is called
        # goals to be fulfilled and operational in general (status ok, communication works)
        self._operational_goals = [x for x in self._goals if x.enabled and x.active]
        self._operational_behaviours = [x for x in self._behaviours if x.operational]

        ### use the symbolic planner if necessary ###
        if plan_if_necessary:
            self._plan_if_necessary()
        self.activation_algorithm.step_preparation()
        ### log behaviour stuff ###
        rhbplog.logdebug("########## BEHAVIOUR  STATES ##########")
        for behaviour in self._behaviours:
            ### do the activation computation ###
            self.activation_algorithm.compute_behaviour_activation_step(ref_behaviour=behaviour)
            # rhbplog.logdebug("%s", behaviour.name)
            # rhbplog.logdebug("\toperational %s", behaviour.operational)
            # rhbplog.logdebug("\twishes %s", behaviour.wishes)
            # rhbplog.logdebug(
            #     "\texecutable: {0} ({1})\n".format(behaviour.executable, behaviour.preconditionSatisfaction))

        self.calculate_final_behaviour_activations()
        rhbplog.loginfo("current activation threshold: %f", self._activationThreshold)

    def calculate_final_behaviour_activations(self):
        ### commit the activation computed in this step ###
        for behaviour in self._behaviours:
            self.activation_algorithm.commit_behaviour_activation(ref_behaviour=behaviour)
            rhbplog.logdebug("activation of %s after this step: %f", behaviour.name, behaviour.activation)

    def handle_interfering_correlations(self, behaviour, currently_influenced_sensors):
        """
        Method checks and resolves (if possible) conflicts with other behaviours of a given behaviour
        :param behaviour: the behaviour that is about to be started
        :param currently_influenced_sensors: list of the currently influenced sensors, will be updated if necessary
        :return: tuple(True if this behaviour is interfering and this cannot be resolved,
                       maybe updated currently_influenced_sensors,
                       stopped_behaviour_amount)
        """
        # TODO .get_pddl_effect_name() might not be 100% accurate, reconsider this

        stopped_behaviour_amount = 0

        interfering_correlations = currently_influenced_sensors.intersection(
            set([item.get_pddl_effect_name() for item in behaviour.correlations]))

        # it must not conflict with an already running behaviour ...
        if len(interfering_correlations) > 0 and not behaviour.manualStart:

            # it might be the case that the conflicting running behaviour(s) has/have less priority/activation ...
            already_running_behaviours_related_to_conflict = filter(
                lambda x: len(interfering_correlations.intersection(
                    set([item.get_pddl_effect_name() for item in x.correlations]))) > 0, self.__executedBehaviours)

            # This is true as long as there are no Aggregators (otherwise those behaviours must have been in conflict
            # with each other).
            assert len(already_running_behaviours_related_to_conflict) <= 1

            # This implementation deals with a list although it is clear that there is at most one element.
            stoppable_behaviours = filter(
                # only if the behaviour has less priority or same with less activation and is interruptable, it should
                # be stopped. Manually started behaviours also cannot be stopped
                lambda x: (x.priority < behaviour.priority or
                           (x.priority == behaviour.priority and x.activation < behaviour.activation))
                          and x.interruptable and not x.manualStart, already_running_behaviours_related_to_conflict)

            # only continue if we can stop ALL offending behaviours. Otherwise we would kill some of them but that
            # doesn't solve the problem and they died for nothing.
            if set(stoppable_behaviours) == set(already_running_behaviours_related_to_conflict):

                rhbplog.loginfo("%s has conflicting correlations with behaviours %s (%s) that can be solved",
                                behaviour.name, already_running_behaviours_related_to_conflict,
                                interfering_correlations)

                for conflictor in stoppable_behaviours:  # stop another behaviour in order to resolve the conflict
                    rhbplog.loginfo("STOP BEHAVIOUR %s because it is interruptable and has less priority or same "
                                    "priority with less activation than %s", conflictor.name, behaviour.name)

                    successfully_stopped = self._stop_behaviour(conflictor, True)

                    # stopping a behaviour here requires to recalculate the currently_influenced_sensors
                    currently_influenced_sensors = self._get_currently_influenced_sensors()
                    # As a result, we have now made room for the higher-priority behaviour ###

                    if successfully_stopped:
                        stopped_behaviour_amount += 1
            else:
                rhbplog.loginfo(
                    "%s will not be started because it has conflicting correlations with already running behaviour(s) "
                    "%s that cannot be solved (%s)",
                    behaviour.name, already_running_behaviours_related_to_conflict, interfering_correlations)
                return True, currently_influenced_sensors, stopped_behaviour_amount
        return False, currently_influenced_sensors, stopped_behaviour_amount

    def _get_currently_influenced_sensors(self):
        """
        Get a set of the currently influenced sensor of all executed behaviours
        :return: set of sensors
        """
        # TODO .get_pddl_effect_name() might not be 100% accurate, reconsider this
        currently_influenced_sensors = set(list(
            itertools.chain.from_iterable(
                [[item.get_pddl_effect_name() for item in x.correlations] for x in self.__executedBehaviours])))
        rhbplog.loginfo("currently influenced sensors: %s", currently_influenced_sensors)
        return currently_influenced_sensors

    def _stop_behaviour(self, behaviour, reset_activation=True):
        """
        stop the execution of a behaviour
        :param behaviour: the behaviour to stop
        :param reset_activation: true or false if the activation of the behaviour should be reset
        """
        if not behaviour.isExecuting:
            return False

        behaviour.stop(reset_activation)
        try:
            self.__executedBehaviours.remove(behaviour)  # remove it from the list of executed behaviours
        except ValueError as e:
            rhbplog.logwarn("Tried to stop already stopped behaviour %s", behaviour.name)
            return False
        rhbplog.logdebug("Stopped %s still running behaviours: %s", behaviour.name, self.__executedBehaviours)
        return True

    def add_goal(self, goal):
        '''
        :param goal: The new goal
        :type goal: AbstractGoalRepresentation
        '''
        with self._step_lock:
            self._goals = filter(lambda x: x.name != goal.name, self._goals)  # kick out existing goals with the same name.
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
            self._behaviours = filter(lambda x: x.name != behaviour.name,
                                      self._behaviours)  # kick out existing behaviours with the same name.

            behaviour.manager = self
            self._behaviours.append(behaviour)
            rhbplog.loginfo("A behaviour with name %s registered(steps=%r)", behaviour.name, behaviour.requires_execution_steps)
            self.__replanningNeeded = True

    def remove_goal(self, goal_name):

        with self._step_lock:
            self._goals = [g for g in self._goals if g.name != goal_name]  # kick out existing goals with that name.
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
        behaviour = Behaviour(name=request.name, planner_prefix=self._prefix, independentFromPlanner=request.independentFromPlanner,
                              requires_execution_steps=request.requiresExecutionSteps,
                              create_log_files=self._create_log_files, log_file_path_prefix=self.__log_file_path_prefix, behaviour_type=request.type)
        self.add_behaviour(behaviour=behaviour)
        return AddBehaviourResponse()

    def pause(self):
        with self._step_lock:  # ensures that the manager is really not just doing something and the next step is blocked
            self.pause_counter += 1
            rhbplog.logdebug('Manager Paused')

    def resume(self):
        with self._step_lock:  # ensures that the manager is really not just doing something and the next step is blocked
            if self.pause_counter > 0:
                self.pause_counter -= 1
                rhbplog.logdebug('Manager Resumed')

    def __pause_callback(self, req):
        self.pause()
        return EmptyResponse()

    def __resume_callback(self, req):
        self.resume()
        return EmptyResponse()

    def __get_paused_callback(self, req):
        return GetPausedResponse(paused=self.paused)

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

    def update_config(self, config):
        """
        Update configuration (e.g. called from dynamic reconfigure
        :param config: dict with the new configuration
        """
        self._activation_threshold_decay = config.get("activationThresholdDecay", self._activation_threshold_decay)

        self._plan_monitoring_all_sensor_changes_by_behaviours = config.get(
            "plan_monitoring_all_sensor_changes_by_behaviours", self._plan_monitoring_all_sensor_changes_by_behaviours)
        self._plan_monitoring_behaviour_missing_influence = config.get(
            "plan_monitoring_behaviour_missing_influence", self._plan_monitoring_behaviour_missing_influence)
        self._plan_monitoring_unexpected_behaviour_finished = config.get(
            "plan_monitoring_unexpected_behaviour_finished", self._plan_monitoring_unexpected_behaviour_finished)

        self.activation_algorithm.update_config(**config)

    def _dynamic_reconfigure_listener_callback(self, config_msg):
        """
        callback for the dynamic_reconfigure update message
        :param config_msg: msg
        """

        config = encoding.decode_config(msg=config_msg)

        self.update_config(config=config)

    def _dynamic_reconfigure_callback(self, config, level):
        """
        direct callback of the dynamic_reconfigure server
        :param config: new config
        :param level:
        :return: adjusted config
        """

        self.update_config(config=config)

        return config
    
    @property
    def activationThreshold(self):
        return self._activationThreshold

    @property
    def executed_behaviours(self):
        return self.__executedBehaviours

    @activationThreshold.setter
    def activationThreshold(self, threshold):
        self._activationThreshold = threshold

    @property
    def goals(self):
        return self._goals

    @property
    def behaviours(self):
        """
        all registered behaviours
        :return: list(Behaviour)
        """
        return self._behaviours

    @property
    def operational_behaviours(self):
        """
        operational behaviours (enabled and properly working)
        :return: list(Behaviour)
        """
        return self._operational_behaviours
    
    @property
    def operational_goals(self):
        """
        operational goals (enabled, not yet fulfilled and properly working)
        :return: list(Behaviour)
        """
        return self._operational_goals

    @property
    def totalActivation(self):
        return self._totalActivation

    @property
    def planExecutionIndex(self):
        return self._planExecutionIndex

    @property
    def enabled(self):
        return self.__enable

    @property
    def paused(self):
        return self.pause_counter > 0
    
    @property
    def plan(self):
        if self._plan:
            return self._plan
        else:
            return None

    @property
    def current_step(self):
        return self._stepCounter

    @property
    def prefix(self):
        return self._prefix

    def disable(self):
        self.__enable = False
        # use while to avoid illegal state of non running behaviors in __executedBehaviors
        while len(self.__executedBehaviours) > 0:
            behaviour = self.__executedBehaviours[0]
            self.__executedBehaviours.remove(behaviour)  # remove it from the list of executed behaviours
            behaviour.stop(True)
        for behaviour in self._behaviours:
            behaviour.reset_activation()

    def enable(self):
        self.__enable = True

    def is_interruptible(self):
        if self.__use_only_running_behaviors_for_interruptible:
            relevant_behaviors = self.__executedBehaviours
        else:
            relevant_behaviors = self._behaviours
        for behavior in relevant_behaviors:
            if not behavior.interruptable:
                return False
        return True

    def plan_with_additional_goal(self, goal_statement):
        """
        Uses the PDDL-planer to make a plan for the last used combination of
        operational goals and one additional goal statement

        If fetchPDDL never was invoked before, it will be here (side effects on
        sensor changes etc (see _fetchPDDL()))

        :param goal_statement: a proper PDDL goal statement
        :type goal_statement: str
        :return: a PDDL plan for currently pursued goals + goal statement
        """

        with self._step_lock:
            if not self.__last_domain_PDDL or len(self._goals) == 0:
                # first get the goals and behaviours we want to use for planning
                behaviours = [x for x in self._behaviours if x.operational]
                # take all goals
                goals = [x for x in self._goals if x.operational]
                domain_pddl = self._fetchPDDL(behaviours=behaviours, goals=goals)
            else:
                domain_pddl = copy(self.__last_domain_PDDL)

            # self.__goalPDDLs[goal][0] is the goalPDDL of goal's (goalPDDL, statePDDL) tuple
            current_goal_conditions = (self.__goalPDDLs[goal][0].statement for goal in self._currently_pursued_goals)
            problem_pddl = self._create_problem_pddl_string(" ".join(current_goal_conditions) + " " + goal_statement)

        plan = self.planner.plan(domain_pddl=domain_pddl, problem_pddl=problem_pddl)
        return plan

    def plan_this_single_goal(self, goal_statement):
        """
        Uses the PDDL-planer to make a plan for exactly one goal statement
        ignoring current goals

        If fetchPDDL never was invoked before, it will be here (side effects on
        sensor changes etc (see _fetchPDDL()))

        :param goal_statement: a proper PDDL goal statement
        :type goal_statement: str
        :return: a PDDL plan for the given goal statement
        """
        with self._step_lock:
            if not self.__last_domain_PDDL or len(self._goals) == 0:
                # first get the goals and behaviours we want to use for planning
                behaviours = [x for x in self._behaviours if x.operational]
                # take all goals
                goals = [x for x in self._goals if x.operational]
                domain_pddl = self._fetchPDDL(behaviours=behaviours, goals=goals)
            else:
                domain_pddl = copy(self.__last_domain_PDDL)

            problem_pddl = self._create_problem_pddl_string(goal_conditions_string=goal_statement)

        plan = self.planner.plan(domain_pddl=domain_pddl, problem_pddl=problem_pddl)
        return plan

    def __plan_with_registered_goals_callback(self, req):
        """
        callback for a service that allows to use the symbolic planner directly for a given set of goals on the current
        state
        :param req: service request. req.goal_names has to include the names of the considered goals
        :return: response with the plan sequence (list of behaviour names)
        """

        response = PlanWithGoalResponse()

        if len(req.goal_names) > 0:

            valid_goals = []
            for goal_name in req.goal_names:

                goal = next((x for x in self._goals if x.name == goal_name), None)
                if goal:
                    valid_goals.append(goal)
                else:
                    rhbplog.logwarn("Trying to plan with an unregistered goal")
            response.plan_sequence = self.plan_with_registered_goals(goals=valid_goals,
                                                                     force_state_update=req.force_state_update)
        else:
            if self._currently_pursued_goals:
                goals = self._currently_pursued_goals
                force_state_update = req.force_state_update
            else:
                # manager did not plan before we just use all enabled(not yet fulfilled) goals
                # and force a state update
                goals = [x for x in self._goals if x.enabled]
                force_state_update = True
            response.plan_sequence = self.plan_with_registered_goals(goals=goals, force_state_update=force_state_update)

        return response

    def plan_with_registered_goals(self, goals, force_state_update=False):
        """
        Planning directly with the symbolic planner and the latest known states (last decision step())
        :param goals: Goal objects that will be used for planning, if list is empty all operational goals are used
        :param force_state_update: If True the state (sensor values) of Goals and Behaviours will be refreshed before
                planning
        :raises RuntimeError if manager has never stepped before
        :return: list(behaviour_names), empty list if no plan could be found
        """
        with self._step_lock:
            if not self.__last_domain_PDDL or force_state_update:
                # first get the goals and behaviours we want to use for planning
                behaviours = [x for x in self._behaviours if x.operational]
                # take all goals if not specified
                goals = goals if len(goals) > 0 else [x for x in self._goals if x.operational]
                domain_pddl = self._fetchPDDL(behaviours=behaviours, goals=goals)
            else:
                domain_pddl = copy(self.__last_domain_PDDL)

            problem_pddl = self._create_problem_pddl(goals)

        try:
            plan = self.planner.plan(domain_pddl, problem_pddl)
            return plan['actions'].values()
        except:
            return []


class ManagerControl(object):
    '''
    Helper class for remotely controlling the manager
    '''

    def __init__(self, planner_prefix=""):
        self.__planner_prefix = planner_prefix

    def pause(self):
        service_name = self.__planner_prefix + '/' + 'Pause'
        rhbplog.logdebug("Waiting for service %s", service_name)
        try:
            rospy.wait_for_service(service_name)
            pauseRequest = rospy.ServiceProxy(service_name, Empty)
            pauseRequest()
        except ROSInterruptException:
            pass

    def resume(self):
        service_name = self.__planner_prefix + '/Resume'
        rhbplog.logdebug("Waiting for service %s", service_name)
        try:
            rospy.wait_for_service(service_name)
            resumeRequest = rospy.ServiceProxy(service_name, Empty)
            resumeRequest()
        except ROSInterruptException:
            pass

    def step(self):
        """
        Triggers a manager step, only works if manager is running on planner node
        """
        service_name = self.__planner_prefix + '/step'
        rhbplog.logdebug("Waiting for service %s", service_name)
        try:
            rospy.wait_for_service(service_name)
            stepRequest = rospy.ServiceProxy(service_name, Empty)
            stepRequest()
        except ROSInterruptException:
            pass

    def set_automatic_stepping(self, enabled):
        """
        Enables/Disables automatic manager stepping, only works if manager is running on planner node
        """
        service_name = self.__planner_prefix + '/set_automatic_stepping'
        rhbplog.logdebug("Waiting for service %s", service_name)
        try:
            rospy.wait_for_service(service_name)
            setAutoSteppingRequest = rospy.ServiceProxy(service_name, SetStepping)
            setAutoSteppingRequest(enabled)
        except ROSInterruptException:
            pass
