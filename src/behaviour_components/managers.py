'''
Created on 23.04.2015

@author: stephan
'''

import rospy
import itertools
from std_srvs.srv import Empty, EmptyResponse
from behaviour_planner.msg import PlannerStatus, Status, Correlation, Wish
from behaviour_planner.srv import AddBehaviour, AddBehaviourResponse, AddGoal, AddGoalResponse, RemoveBehaviour, RemoveBehaviourResponse, RemoveGoal, RemoveGoalResponse, ForceStart, ForceStartResponse, Activate
from behaviour_components.behaviours import Behaviour
from behaviour_components.goals import Goal
from behaviour_components.util import PDDL, mergeStatePDDL, tokenizePDDL, getStatePDDLchanges, predicateRegex
import ffp

class Manager(object):
    '''
    This is the manager class that keeps track of all elements in the network (behaviours, goals, sensors).
    Behaviours need this to know what sensors exist in the world and how they are correlated their measurement.
    They also need to know the goals to decide whether they are supporting them (which increases their activation) or inhibiting them (which decreases their activation).
    And last, they need to know whether another behaviour conflicts with them to reduce the activation of it.
    Also global constants like activation thresholds are stored here.
    '''

    def __init__(self, logLevel = rospy.INFO, **kwargs):
        '''
        Constructor
        '''
        rospy.init_node('behaviourPlannerManager', log_level = logLevel)
        self._prefix = kwargs["prefix"] if "prefix" in kwargs else "" # if you have multiple planners in the same ROS environment use this to distinguish between the instances
        self.__addBehaviourService = rospy.Service(self._prefix + 'AddBehaviour', AddBehaviour, self.__addBehaviour)
        self.__addGoalService = rospy.Service(self._prefix + 'AddGoal', AddGoal, self.__addGoal)
        self.__removeBehaviourService = rospy.Service(self._prefix + 'RemoveBehaviour', RemoveBehaviour, self.__removeBehaviour)
        self.__removeGoalService = rospy.Service(self._prefix + 'RemoveGoal', RemoveGoal, self.__removeGoal)
        self.__manualStartService = rospy.Service(self._prefix + 'ForceStart', ForceStart, self.__manualStart)
        self.__pauseService = rospy.Service(self._prefix + 'Pause', Empty, self.__pauseCallback)
        self.__resumeService = rospy.Service(self._prefix + 'Resume', Empty, self.__resumeCallback)
        self.__statusPublisher = rospy.Publisher('/' + self._prefix + 'Planner/plannerStatus', PlannerStatus, queue_size=1)
        self._sensors = []
        self._goals = []
        self._activeGoals = [] # pre-computed (in step()) list of operational goals
        self._behaviours = []
        self._activeBehaviours = [] # pre-computed (in step()) list of operational behaviours
        self._totalActivation = 0.0 # pre-computed (in step()) sum all activations of active behaviours
        self._activationThreshold = kwargs["activationThreshold"] if "activationThreshold" in kwargs else rospy.get_param("activationThreshold", 7.0) # not sure how to set this just yet.
        self._activationDecay = kwargs["activationDecay"] if "activationDecay" in kwargs else rospy.get_param("activationDecay", .9) # not sure how to set this just yet.
        self._stepCounter = 0
        self.__threshFile = open("threshold.log", 'w')
        self.__threshFile.write("{0}\t{1}\n".format("Time", "activationThreshold"))
        self.__running = True # toggled by the pause and resume services
        self.__replanningNeeded = False # this is set when behaviours or goals are added or removed, or the last planning attempt returned an error.
        self.__previousStatePDDL = PDDL()
        self.__sensorChanges = {} # this dictionary stores all sensor changes between two steps in the form {sensor name <string> : indicator <float>}. Note: the float is not scaled [-1.0 to 1.0] but shows a direction (positive or negative).
        self._plan = {}
        self._planExecutionIndex = 0
        self.__goalPDDLs = {}
        
    def __del__(self):
        self.__addBehaviourService.shutdown()
        self.__addGoalService.shutdown()
        self.__removeBehaviourService.shutdown()
        self.__removeGoalService.shutdown()
        self.__manualStartService.shutdown()
        self.__statusPublisher.unregister()
        self.__threshFile.close()
    
    def fetchPDDL(self):
        '''
        This method fetches the PDDL from all behaviours and goals, merges the state descriptions and returns a tuple of
        (domainPDDL, problemPDDL) strings ready for ff.
        As a side effect, is also computes the sensor changes that happened between the last invocation and now.
        '''
        behaviourPDDLs = [behaviour.fetchPDDL() for behaviour in self._activeBehaviours]
        self.__goalPDDLs = {goal: goal.fetchPDDL() for goal in self._activeGoals}
        pddl = PDDL()
        for actionPDDL, _statePDDL in behaviourPDDLs:
            pddl.statement += actionPDDL.statement
            pddl.predicates = pddl.predicates.union(actionPDDL.predicates)
            pddl.functions = pddl.functions.union(actionPDDL.functions)
        domainPDDLString = "(define (domain {0})\n".format(self._prefix)
        domainPDDLString += "(:predicates\n    " + "\n    ".join("({0})".format(x) for x in pddl.predicates) + ")\n"
        domainPDDLString += "(:functions\n    " + "\n    ".join("({0})".format(x) for x in pddl.functions) + ")\n"
        domainPDDLString += pddl.statement + ")"
        mergedStatePDDL = PDDL()
        for _actionPDDL, statePDDL in behaviourPDDLs:
            mergedStatePDDL = mergeStatePDDL(statePDDL, mergedStatePDDL)
        for _goalPDDL, statePDDL in self.__goalPDDLs.values():
            mergedStatePDDL = mergeStatePDDL(statePDDL, mergedStatePDDL)
        # filter out negative predicates. FF can't handle them!
        statePDDL = PDDL(statement = "\n\t\t".join(filter(lambda x: predicateRegex.match(x) is None or predicateRegex.match(x).group(2) is None, tokenizePDDL(mergedStatePDDL.statement))))# if the regex does not match it is a function (which is ok) and if the second group is None it is not negated (which is also ok)
        rospy.logdebug("statePDDL %s", statePDDL.statement)
        # debugging only
        with open("robotDomain{0}.pddl".format(self._stepCounter), 'w') as outfile:
            outfile.write(domainPDDLString)
        # compute changes
        self.__sensorChanges = getStatePDDLchanges(self.__previousStatePDDL, statePDDL)
        self.__previousStatePDDL = statePDDL
        return domainPDDLString
    
    def createProblemPDDL(self, goals):
        '''
        This method creates the problem PDDL for a given set of goals.
        It relies on the fact that self.fetchPDDL() has run before and filled the self.__goalPDDLs dictionary with the most recent responses from the actual goals and self.__previousStatePDDL with the CURRENT state PDDL
        '''
        goalConditions = (self.__goalPDDLs[goal][0].statement for goal in goals) # self.__goalPDDLs[goal][0] is the goalPDDL of goal's (goalPDDL, statePDDL) tuple
        problemPDDLString = "(define (problem problem-{0})\n\t(:domain {0})\n\t(:init \n\t\t(= (costs) 0)\n\t\t{1}\n\t)\n".format(self._prefix, self.__previousStatePDDL.statement) # at this point the "previous" is the current state PDDL
        problemPDDLString += "\t(:goal (and {0}))\n\t(:metric minimize (costs))\n".format(" ".join(goalConditions))
        problemPDDLString += ")\n"
        # debugging only
        with open("robotProblem{0}.pddl".format(self._stepCounter), 'w') as outfile:
            outfile.write(problemPDDLString)
        return problemPDDLString
    
    def priorityGoalSequences(self):
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
    
    def planIfNecessary(self):
        '''
        this method plans using the symbolic planner it it is required to do so.
        Replanning is required whenever any or many of the following conditions is/are met:
        1) Behaviours or Goals have been added or removed
        2) The state of the world has changed an this change is not caused by the next behaviour in the plan
        3) An unexpected behaviour was running (not head of the plan)
        4) Nothing happened for a long time <-- this is still TBD
        5) The last planning attempt was unsuccessful
        '''
        domainPDDL = self.fetchPDDL() # this also updates our self.__sensorChanges and self.__goalPDDLs dictionaries
        # now check whether we expected the world to change so by comparing the observed changes to the correlations of the running behaviours
        changesWereExpected = True
        for sensorName, indicator in self.__sensorChanges.iteritems():
            changeWasExpected = False
            for behaviour in self.__executedBehaviours:
                for item in behaviour.correlations:
                    if item[0] == sensorName and item[1] * indicator > 0: # the observed change happened because of the running behaviour (at least the behaviour is correlated to the changed sensor in the correct way)
                        changeWasExpected = True
                        break
                if changeWasExpected:
                    break
            if not changeWasExpected:
                changesWereExpected = False
                break
        # the next part is a little plan execution monitoring
        # it tracks progress on the plan and finds out if something unexpected finished. FIXME: there might be behaviours like collision avoidance the are expected to run alongside with the planned behaviours.
        unexpectedBehaviourFinished = False
        # make sure the finished behaviour was part of the plan at all (otherwise it is unexpected)
        if self._plan:
            for behaviour in self.__executedBehaviours:
                if behaviour.justFinished and behaviour.name not in [name for index, name in self._plan["actions"].iteritems() if index >= self._planExecutionIndex]:
                    unexpectedBehaviourFinished = True # it was unexpected
            if not unexpectedBehaviourFinished: # if we detected a deviation from the plan we can already stop here
                for index in filter(lambda x: x >= self._planExecutionIndex, sorted(self._plan["actions"].keys())): # walk along the remaining plan
                    for behaviour in self.__executedBehaviours: # only those may be finished. the others were not even running
                        if behaviour.name == self._plan["actions"][index] and behaviour.justFinished:
                            if self._planExecutionIndex == index: # if it was a planned behaviour and it finished
                                self._planExecutionIndex += 1 # we are one step ahead in our plan
                            else: # or otherwise
                                unexpectedBehaviourFinished = True # it was unexpected
        # now, we know whether we need to plan again or not
        if self.__replanningNeeded or unexpectedBehaviourFinished or not changesWereExpected:
            rospy.loginfo("### PLANNING ### because\nreplanning was needed: %s\nchanges were unexpected: %s\nunexpected behaviour finished: %s", self.__replanningNeeded, not changesWereExpected, unexpectedBehaviourFinished)
            # now we need to make the best of our planning problem:
            # In cases where the full set of goals can't be reached because the planner does not find a solution a reduced set should be used.
            # The reduction will eliminate goals of inferiour priority until the highest priority goal is tried alone.
            # If that cannot be reached the search goes backwards and tries all other goals with lower priorities in descending order until a reachable goal is found.
            for goalSequence in self.priorityGoalSequences():
                try:
                    rospy.logdebug("trying to reach goals %s", goalSequence)
                    problemPDDL = self.createProblemPDDL(goalSequence)
                    tmpPlan = ffp.plan(domainPDDL, problemPDDL)
                    if tmpPlan and "cost" in tmpPlan and tmpPlan["cost"] != -1.0:
                        rospy.loginfo("FOUND PLAN: %s", tmpPlan)
                        self._plan = tmpPlan
                        self.__replanningNeeded = False
                        self._planExecutionIndex = 0
                        break
                    else:
                        rospy.loginfo("PROBLEM IMPOSSIBLE")
                except Exception as e:
                    rospy.logerr("PLANNER ERROR: %s", e)
                    self.__replanningNeeded = True # in case of planning exceptions try again next iteration
        else:
            rospy.loginfo("### NOT PLANNING ###\nbecause replanning was needed: %s\nchanges were unexpected: %s\nunexpected behaviour finished: %s\n current plan execution index: %s", self.__replanningNeeded, not changesWereExpected, unexpectedBehaviourFinished, self._planExecutionIndex)
    
    def step(self):
        if not self.__running:
            return
        plannerStatusMessage = PlannerStatus()
        self.__threshFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._activationThreshold))
        self.__threshFile.flush()
        plannerStatusMessage.activationThreshold = self._activationThreshold
        self._totalActivation = 0.0
        rospy.loginfo("###################################### STEP {0} ######################################".format(self._stepCounter))
        ### collect information about behaviours ###
        for behaviour in self._behaviours:
            behaviour.fetchStatus()
            if behaviour.active:
                self._totalActivation += behaviour.activation
        if self._totalActivation == 0.0:
            self._totalActivation = 1.0 # the behaviours are going to divide by this so make sure it is non-zero
        rospy.logdebug("############# GOAL STATI #############")
        ### collect information about goals ###
        for goal in self._goals:
            goal.fetchStatus()
            statusMessage = Status()
            statusMessage.name = goal.name
            statusMessage.wishes = [Wish(sensorName, indicator) for (sensorName, indicator) in goal.wishes]
            statusMessage.active = goal.active
            statusMessage.activated = goal.activated
            statusMessage.satisfaction = goal.fulfillment
            statusMessage.priority = goal.priority
            plannerStatusMessage.goals.append(statusMessage)
            rospy.logdebug("%s: active: %s, fulfillment: %f, wishes %s", goal.name, goal.active, goal.fulfillment, goal.wishes)
            if goal.active and not goal.isPermanent and goal.fulfillment >= 1:
                rospy.logdebug("Waiting for service %s", goal.name + 'Activate')
                rospy.wait_for_service(goal.name + 'Activate')
                activateRequest = rospy.ServiceProxy(goal.name + 'Activate', Activate)
                activateRequest(False)
                rospy.logdebug("Set Activated of %s goal to False", goal.name)
        ### do housekeeping ###
        self._activeGoals = filter(lambda x: x.active, self._goals)
        self._activeBehaviours = filter(lambda x: x.active, self._behaviours) # this line (and the one above) must happen BEFORE computeActivation() of the behaviours is called in each step.
        ### use the symbolic planner if necessary ###
        self.planIfNecessary()
        ### log behaviour stuff ###
        rospy.logdebug("########## BEHAVIOUR  STUFF ##########")
        for behaviour in self._behaviours:
            rospy.logdebug("%s", behaviour.name)
            rospy.logdebug("\tactive %s", behaviour.active)
            rospy.logdebug("\twishes %s", behaviour.wishes)
            rospy.logdebug("\tactivation from preconditions: %s", behaviour.activationFromPreconditions)
            if behaviour.active:
                rospy.logdebug("\tactivation from goals: %s", behaviour.getActivationFromGoals(logging = True))
                rospy.logdebug("\tinhibition from goals: %s", behaviour.getInhibitionFromGoals(logging = True))
                rospy.logdebug("\tactivation from predecessors: %s", behaviour.getActivationFromPredecessors(logging = True))
                rospy.logdebug("\tactivation from successors: %s", behaviour.getActivationFromSuccessors(logging = True))
                rospy.logdebug("\tinhibition from conflicted: %s", behaviour.getInhibitionFromConflicted(logging = True))
                rospy.logdebug("\tactivation from plan: %s", behaviour.getActivationFromPlan(logging = True))
            rospy.logdebug("\texecutable: {0} ({1})".format(behaviour.executable, behaviour.preconditionSatisfaction))
            ### do the activation computation ###
            behaviour.computeActivation()
        ### commit the activation computed in this step ###
        for behaviour in self._behaviours:
            behaviour.commitActivation()
            rospy.logdebug("activation of %s after this step: %f", behaviour.name, behaviour.activation)
        rospy.loginfo("current activation threshold: %f", self._activationThreshold)
        rospy.loginfo("############## ACTIONS ###############")
        self.__executedBehaviours = filter(lambda x: x.isExecuting, self._behaviours) # actually, activeBehaviours should be enough as search space but if the behaviour implementer resets active before isExecuting we are safe this way
        currentlyInfluencedSensors = set(list(itertools.chain.from_iterable([[item[0] for item in x.correlations] for x in self.__executedBehaviours])))
        rospy.loginfo("currently running behaviours: %s", self.__executedBehaviours)
        rospy.loginfo("currently influenced sensors: %s", currentlyInfluencedSensors)
        for behaviour in sorted(self._behaviours, key = lambda x: x.activation, reverse = True):
            statusMessage = Status()
            statusMessage.name = behaviour.name
            statusMessage.activation = behaviour.activation
            statusMessage.satisfaction = behaviour.preconditionSatisfaction
            statusMessage.isExecuting = behaviour.isExecuting
            statusMessage.progress = behaviour.progress
            statusMessage.executable = behaviour.executable
            statusMessage.threshold = behaviour.readyThreshold
            statusMessage.priority = behaviour.priority
            statusMessage.interruptable = behaviour.interruptable
            statusMessage.activated = behaviour.activated
            statusMessage.active = behaviour.active
            statusMessage.correlations = [Correlation(sensorName, value) for (sensorName, value) in behaviour.correlations]
            statusMessage.wishes = [Wish(sensorName, indicator) for (sensorName, indicator) in behaviour.wishes]
            plannerStatusMessage.behaviours.append(statusMessage)
            ### now comes a series of tests that a behaviour must pass in order to get started ###
            if not behaviour.active and not behaviour.manualStart: # it must be active
                rospy.loginfo("%s will not be started because it is not active", behaviour.name)
                continue
            if behaviour.isExecuting: # it must not already run
                rospy.loginfo("%s will not be started because it is already executing", behaviour.name)
                continue
            if not behaviour.executable and not behaviour.manualStart: # it must be executable
                rospy.loginfo("%s will not be started because it is not executable", behaviour.name)
                continue
            if behaviour.activation < self._activationThreshold and not behaviour.manualStart: # it must have high-enough activation
                rospy.loginfo("%s will not be started because it has not enough activation (%f < %f)", behaviour.name, behaviour.activation, self._activationThreshold)
                continue
            interferingCorrelations = currentlyInfluencedSensors.intersection(set([item[0] for item in behaviour.correlations]))
            if len(interferingCorrelations) > 0 and not behaviour.manualStart: # it must not conflict with an already running behaviour ...
                alreadyRunningBehavioursRelatedToConflict = filter(lambda x: len(interferingCorrelations.intersection(set([item[0] for item in x.correlations]))) > 0, self.__executedBehaviours)  # but it might be the case that the conflicting running behaviour(s) has/have less priority ...
                assert len(alreadyRunningBehavioursRelatedToConflict) <= 1 # This is true as long as there are no Aggregators (otherwise those behaviours must have been in conflict with each other). TODO: remove this assertion in case Aggregators are added
                # This implementation deals with a list although it it clear that there is at most one element.
                # With Aggregators this is not necessarily the case: There may be multiple behaviours running and affecting the same Aggregator, hence being correlated to the same sensor so they could all appear in this list.
                stoppableBehaviours = filter(lambda x: x.priority < behaviour.priority and x.interruptable and not x.manualStart, alreadyRunningBehavioursRelatedToConflict) # only if the behaviour has less priority and is interruptable it should be stopped. Manually started behaviours also cannot be stopped
                if set(stoppableBehaviours) == set(alreadyRunningBehavioursRelatedToConflict): # only continue if we can stop ALL offending behaviours. Otherwise we would kill some of them but that doesn't solve the problem and they died for nothing.
                    rospy.loginfo("%s has conflicting correlations with behaviours %s (%s) that can be solved", behaviour.name, alreadyRunningBehavioursRelatedToConflict, interferingCorrelations)
                    for conflictor in stoppableBehaviours:
                        rospy.loginfo("STOP BEHAVIOUR %s because it is interruptable and has less priority than %s", behaviour.name, conflictor.name)
                        conflictor.stop()
                        self.__executedBehaviours.remove(conflictor) # remove it from the list of executed behaviours
                        currentlyInfluencedSensors = currentlyInfluencedSensors.difference(set([item[0] for item in conflictor.correlations])) # remove all its correlations from the set of currently affected sensors
                        rospy.loginfo("still running behaviours: %s", self.__executedBehaviours)
                        rospy.loginfo("updated influenced sensors: %s", currentlyInfluencedSensors)
                    ### we have now made room for the higher-priority behaviour ###
                else:
                    rospy.loginfo("%s will not be started because it has conflicting correlations with already running behaviour(s) %s that cannot be solved (%s)", behaviour.name, alreadyRunningBehavioursRelatedToConflict, interferingCorrelations)
                    continue
            ### if the behaviour got here it really is ready to be started ###
            rospy.loginfo("START BEHAVIOUR %s", behaviour.name)
            if behaviour.manualStart:
                rospy.loginfo("BEHAVIOUR %s WAS STARTED BECAUSE OF MANUAL REQUEST")
            behaviour.start()
            rospy.loginfo("INCREASING ACTIVATION THRESHOLD TO %f", self._activationThreshold)
            self._activationThreshold *= (1 / rospy.get_param("activationThresholdDecay", .8))
            currentlyInfluencedSensors = currentlyInfluencedSensors.union(set([item[0] for item in behaviour.correlations]))
            self.__executedBehaviours.append(behaviour)
            rospy.loginfo("now running behaviours: %s", self.__executedBehaviours)
            rospy.loginfo("updated influenced sensors: %s", currentlyInfluencedSensors)
        plannerStatusMessage.runningBehaviours = map(lambda x: x.name, self.__executedBehaviours)
        plannerStatusMessage.influencedSensors = currentlyInfluencedSensors
        if len(self.__executedBehaviours) == 0 and len(self._activeBehaviours) > 0:
            self._activationThreshold *= rospy.get_param("activationThresholdDecay", .8)
            rospy.loginfo("REDUCING ACTIVATION THRESHOLD TO %f", self._activationThreshold)
        plannerStatusMessage.activationThresholdDecay = rospy.get_param("activationThresholdDecay", .8) # TODO retrieve rosparam only once
        self.__statusPublisher.publish(plannerStatusMessage)
        self._stepCounter += 1
    
    #TODO all those operations are potentially dangerous while the above step() method is running (especially the remove stuff)
    def __addGoal(self, request):
        self._goals = filter(lambda x: x.name != request.name, self._goals) # kick out existing goals with the same name. 
        goal = Goal(request.name, request.permanent)
        self._goals.append(goal)
        rospy.loginfo("A goal with name %s registered", goal.name)
        self.__replanningNeeded = True;
        return AddGoalResponse()
    
    def __addBehaviour(self, request):
        self._behaviours = filter(lambda x: x.name != request.name, self._behaviours) # kick out existing behaviours with the same name.
        behaviour = Behaviour(request.name)
        behaviour.manager = self
        behaviour.activationDecay = self._activationDecay
        self._behaviours.append(behaviour)
        rospy.loginfo("A behaviour with name %s registered", behaviour.name)
        self.__replanningNeeded = True;
        return AddBehaviourResponse()
    
    def __pauseCallback(self, dummy):
        self.__running = False
        return EmptyResponse()
    
    def __resumeCallback(self, dummy):
        self.__running = True
        return EmptyResponse()
    
    def __removeGoal(self, request):
        self._goals = filter(lambda x: x.name != request.name, self._goals) # kick out existing goals with that name.
        self.__replanningNeeded = True;
        return RemoveGoalResponse()
    
    def __removeBehaviour(self, request):
        self._behaviours = filter(lambda x: x.name != request.name, self._behaviours) # kick out existing behaviours with the same name.
        self.__replanningNeeded = True;
        return RemoveBehaviourResponse()
    
    def __manualStart(self, request):
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
    def activationDecay(self):
        return self._activationDecay
    
    @activationDecay.setter
    def activationDecay(self, rate):
        self._activationDecay = rate
    
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
    def plan(self):
        if self._plan:
            return self._plan
        else:
            return None
    
