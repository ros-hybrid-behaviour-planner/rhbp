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

class Manager(object):
    '''
    This is the manager class that keeps track of all elements in the network (behaviours, goals, sensors).
    Behaviours need this to know what sensors exist in the world and how they are correlated their measurement.
    They also need to know the goals to decide whether they are supporting them (which increases their activation) or inhibiting them (which decreases their activation).
    And last, they need to know whether another behaviour conflicts with them to reduce the activation of it.
    Also global constants like activation thresholds are stored here.
    '''

    def __init__(self, **kwargs):
        '''
        Constructor
        '''
        rospy.init_node('behaviourPlannerManager', anonymous=True, log_level=rospy.DEBUG)
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
        self._activationThreshold = kwargs["activationThreshold"] if "activationThreshold" in kwargs else rospy.get_param("activationThreshold", 7.0) # not sure how to set this just yet.
        self._activationDecay = kwargs["activationDecay"] if "activationDecay" in kwargs else rospy.get_param("activationDecay", .9) # not sure how to set this just yet.
        self._stepCounter = 0
        self.__threshFile = open("threshold.log", 'w')
        self.__threshFile.write("{0}\t{1}\n".format("Time", "activationThreshold"))
        self.__running = True # toggled by the pause and resume services
        
    def __del__(self):
        self.__addBehaviourService.shutdown()
        self.__addGoalService.shutdown()
        self.__removeBehaviourService.shutdown()
        self.__removeGoalService.shutdown()
        self.__manualStartService.shutdown()
        self.__statusPublisher.unregister()
        self.__threshFile.close()
    
    def fetchPDDL(self):
        with open("robotDomain.pddl", 'a') as outfile:
            for behaviour in self._behaviours:
                outfile.write(behaviour.fetchPDDL())
    
    def step(self):
        if not self.__running:
            return
        plannerStatusMessage = PlannerStatus()
        self.__threshFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._activationThreshold))
        self.__threshFile.flush()
        plannerStatusMessage.activationThreshold = self._activationThreshold
        rospy.loginfo("###################################### STEP {0} ######################################".format(self._stepCounter))
        ### collect information about behaviours ###
        for behaviour in self._behaviours:
            behaviour.fetchStatus()
        rospy.logdebug("############# GOAL STATI #############")
        ### collect information about goals ###
        for goal in self._goals:
            goal.fetchStatus()
            statusMessage = Status()
            statusMessage.name = goal.name
            statusMessage.wishes = [Wish(name, indicator) for (name, indicator) in goal.wishes.iteritems()]
            statusMessage.active = goal.active
            statusMessage.activated = goal.activated
            statusMessage.satisfaction = goal.fulfillment
            plannerStatusMessage.goals.append(statusMessage)
            rospy.logdebug("%s: active: %s, fulfillment: %f, wishes %s", goal.name, goal.active, goal.fulfillment, goal.wishes)
            if goal.active and not goal.isPermanent and goal.fulfillment >= 1:
                rospy.logdebug("Waiting for service %s", goal.name + 'Activate')
                rospy.wait_for_service(goal.name + 'Activate')
                activateRequest = rospy.ServiceProxy(goal.name + 'Activate', Activate)
                activateRequest(False)
                rospy.logdebug("Set Activated of %s goal to False", goal.name)
        #### do housekeeping ###
        self._activeGoals = filter(lambda x: x.active, self._goals)
        self._activeBehaviours = filter(lambda x: x.active, self._behaviours) # this line (and the one above) must happen BEFORE computeActivation() of the behaviours is called in each step.
        ### log behaviour stuff ###
        rospy.logdebug("########## BEHAVIOUR  STUFF ##########")
        for behaviour in self._behaviours:
            rospy.logdebug("%s", behaviour.name)
            rospy.logdebug("\tactive %s", behaviour.active)
            rospy.logdebug("\twishes %s", behaviour.wishes)
            rospy.logdebug("\tactivation from preconditions: %s", behaviour.activationFromPreconditions)
            if behaviour.active:
                rospy.logdebug("\tactivation from goals: %s", behaviour.getActivationFromGoals())
                rospy.logdebug("\tinhibition from goals: %s", behaviour.getInhibitionFromGoals())
                rospy.logdebug("\tactivation from predecessors: %s", behaviour.getActivationFromPredecessors())
                rospy.logdebug("\tactivation from successors: %s", behaviour.getActivationFromSuccessors())
                rospy.logdebug("\tinhibition from conflicted: %s", behaviour.getInhibitionFromConflicted())
            rospy.logdebug("\texecutable: {0} ({1})".format(behaviour.executable, behaviour.preconditionSatisfaction))
            ### do the activation computation ###
            behaviour.computeActivation()
        ### commit the activation computed in this step ###
        for behaviour in self._behaviours:
            behaviour.commitActivation()
            rospy.logdebug("activation of %s after this step: %f", behaviour.name, behaviour.activation)
        rospy.loginfo("current activation threshold: %f", self._activationThreshold)
        rospy.loginfo("############## ACTIONS ###############")
        executedBehaviours = filter(lambda x: x.isExecuting, self._behaviours) # actually, activeBehaviours should be enough as search space but if the behaviour implementer resets active before isExecuting we are safe this way
        currentlyInfluencedSensors = set(list(itertools.chain.from_iterable([x.correlations.keys() for x in executedBehaviours])))
        rospy.loginfo("currently running behaviours: %s", executedBehaviours)
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
            statusMessage.correlations = [Correlation(name, value) for (name, value) in behaviour.correlations.iteritems()]
            statusMessage.wishes = [Wish(name, indicator) for (name, indicator) in behaviour.wishes.iteritems()]
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
            interferingCorrelations = currentlyInfluencedSensors.intersection(set(behaviour.correlations.keys()))
            if len(interferingCorrelations) > 0 and not behaviour.manualStart: # it must not conflict with an already running behaviour ...
                alreadyRunningBehavioursRelatedToConflict = filter(lambda x: len(interferingCorrelations.intersection(set(x.correlations.keys()))) > 0, executedBehaviours)  # but it might be the case that the conflicting running behaviour(s) has/have less priority ...
                assert len(alreadyRunningBehavioursRelatedToConflict) <= 1 # This is true as long as there are no Aggregators (otherwise those behaviours must have been in conflict with each other). TODO: remove this assertion in case Aggregators are added
                # This implementation deals with a list although it it clear that there is at most one element.
                # With Aggregators this is not necessarily the case: There may be multiple behaviours running and affecting the same Aggregator, hence being correlated to the same sensor so they could all appear in this list.
                stoppableBehaviours = filter(lambda x: x.priority < behaviour.priority and x.interruptable and not x.manualStart, alreadyRunningBehavioursRelatedToConflict) # only if the behaviour has less priority and is interruptable it should be stopped. Manually started behaviours also cannot be stopped
                if set(stoppableBehaviours) == set(alreadyRunningBehavioursRelatedToConflict): # only continue if we can stop ALL offending behaviours. Otherwise we would kill some of them but that doesn't solve the problem and they died for nothing.
                    rospy.loginfo("%s has conflicting correlations with behaviours %s (%s) that can be solved", behaviour.name, alreadyRunningBehavioursRelatedToConflict, interferingCorrelations)
                    for conflictor in stoppableBehaviours:
                        rospy.loginfo("STOP BEHAVIOUR %s because it is interruptable and has less priority than %s", behaviour.name, conflictor.name)
                        conflictor.stop()
                        executedBehaviours.remove(conflictor) # remove it from the list of executed behaviours
                        currentlyInfluencedSensors = currentlyInfluencedSensors.difference(set(conflictor.correlations.keys())) # remove all its correlations from the set of currently affected sensors
                        rospy.loginfo("still running behaviours: %s", executedBehaviours)
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
            currentlyInfluencedSensors = currentlyInfluencedSensors.union(set(behaviour.correlations.keys()))
            executedBehaviours.append(behaviour)
            rospy.loginfo("now running behaviours: %s", executedBehaviours)
            rospy.loginfo("updated influenced sensors: %s", currentlyInfluencedSensors)
        plannerStatusMessage.runningBehaviours = map(lambda x: x.name, executedBehaviours)
        plannerStatusMessage.influencedSensors = currentlyInfluencedSensors
        if len(executedBehaviours) == 0 and len(self._activeBehaviours) > 0:
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
        return AddGoalResponse()
    
    def __addBehaviour(self, request):
        self._behaviours = filter(lambda x: x.name != request.name, self._behaviours) # kick out existing behaviours with the same name.
        behaviour = Behaviour(request.name)
        behaviour.manager = self
        behaviour.activationDecay = self._activationDecay
        self._behaviours.append(behaviour)
        rospy.loginfo("A behaviour with name %s registered", behaviour.name)
        return AddBehaviourResponse()
    
    def __pauseCallback(self, dummy):
        self.__running = False
        return EmptyResponse()
    
    def __resumeCallback(self, dummy):
        self.__running = True
        return EmptyResponse()
    
    def __removeGoal(self, request):
        self._goals = filter(lambda x: x.name != request.name, self._goals) # kick out existing goals with that name. 
        return RemoveGoalResponse()
    
    def __removeBehaviour(self, request):
        self._behaviours = filter(lambda x: x.name != request.name, self._behaviours) # kick out existing behaviours with the same name.
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
    
    
