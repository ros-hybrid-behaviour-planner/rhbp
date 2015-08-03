'''
Created on 23.04.2015

@author: stephan
'''

import rospy
import itertools
from behaviourPlannerPython.srv import AddBehaviour, AddBehaviourResponse, AddGoal, AddGoalResponse, RemoveBehaviour, RemoveBehaviourResponse, RemoveGoal, RemoveGoalResponse
from buildingBlocks.behaviours import Behaviour
from buildingBlocks.goals import Goal

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
        rospy.init_node('behaviourPlannerManager', anonymous=True, log_level=rospy.INFO)
        self._prefix = kwargs["prefix"] if "prefix" in kwargs else "" # if you have multiple planners in the same ROS environment use this to distinguish between the instances
        self.__addBehaviourService = rospy.Service(self._prefix + 'AddBehaviour', AddBehaviour, self.__addBehaviour)
        self.__addGoalService = rospy.Service(self._prefix + 'AddGoal', AddGoal, self.__addGoal)
        self.__removeBehaviourService = rospy.Service(self._prefix + 'RemoveBehaviour', RemoveBehaviour, self.__removeBehaviour)
        self.__removeGoalService = rospy.Service(self._prefix + 'RemoveGoal', RemoveGoal, self.__removeGoal)
        self._sensors = []
        self._goals = []
        self._activeGoals = [] # pre-computed (in step()) list of operational goals
        self._behaviours = []
        self._activeBehaviours = [] # pre-computed (in step()) list of operational behaviours
        self._activationDecay = .9     # not sure how to set this just yet.
        self._activationThreshold = kwargs["activationThreshold"] if "activationThreshold" in kwargs else 7.0 # not sure how to set this just yet.
        self._activationDecay = kwargs["activationDecay"] if "activationDecay" in kwargs else .9  # not sure how to set this just yet.
        self._stepCounter = 0
        self.__threshFile = open("threshold.log", 'w')
        self.__threshFile.write("{0}\t{1}\n".format("Time", "activationThreshold"))
        
    def __del__(self):
        self.__addBehaviourService.shutdown()
        self.__addGoalService.shutdown()
        self.__removeBehaviourService.shutdown()
        self.__removeGoalService.shutdown()
        self.__threshFile.close()
    
    def step(self):
        self.__threshFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._activationThreshold))
        self.__threshFile.flush()
        rospy.loginfo("###################################### STEP {0} ######################################".format(self._stepCounter))
        ### collect information about behaviours ###
        for behaviour in self._behaviours:
            behaviour.fetchStatus()
        rospy.loginfo("############# GOAL STATI #############")
        ### collect information about goals ###
        for goal in self._goals:
            goal.fetchStatus()
            rospy.loginfo("%s: active: %s, fulfillment: %f, wishes %s", goal.name, goal.active, goal.fulfillment, goal.wishes)
        #### do housekeeping ###
        self._goals = filter(lambda x: x.isPermanent or x.fulfillment < 1.0 or not x.active, self._goals) # remove non-permanent goals that were achieved
        self._activeGoals = filter(lambda x: x.active, self._goals)
        self._activeBehaviours = filter(lambda x: x.active, self._behaviours) # this line (and the one above) must happen BEFORE computeActivation() of the behaviours is called in each step.
        ### log behaviour stuff ###
        rospy.loginfo("########## BEHAVIOUR  STUFF ##########")
        for behaviour in self._behaviours:
            rospy.loginfo("%s", behaviour.name)
            rospy.loginfo("\tactive %s", behaviour.active)
            rospy.loginfo("\twishes %s", behaviour.wishes)
            rospy.loginfo("\tactivation from preconditions: %s", behaviour.activationFromPreconditions)
            if behaviour.active:
                rospy.loginfo("\tactivation from goals: %s", behaviour.getActivationFromGoals())
                rospy.loginfo("\tinhibition from goals: %s", behaviour.getInhibitionFromGoals())
                rospy.loginfo("\tactivation from predecessors: %s", behaviour.getActivationFromPredecessors())
                rospy.loginfo("\tactivation from successors: %s", behaviour.getActivationFromSuccessors())
                rospy.loginfo("\tinhibition from conflicted: %s", behaviour.getInhibitionFromConflicted())
            rospy.loginfo("\texecutable: {0} ({1})".format(behaviour.executable, behaviour.preconditionSatisfaction))
            ### do the activation computation ###
            behaviour.computeActivation()
        ### commit the activation computed in this step ###
        for behaviour in self._behaviours:
            behaviour.commitActivation()
            rospy.loginfo("activation of %s after this step: %f", behaviour.name, behaviour.activation)
        rospy.loginfo("current activation threshold: %f", self._activationThreshold)
        rospy.loginfo("############## ACTIONS ###############")
        executedBehaviours = filter(lambda x: x.isExecuting, self._behaviours) # actually, activeBehaviours should be enough as search space but if the behaviour implementer resets active before isExecuting we are safe this way
        currentlyInfluencedSensors = set(list(itertools.chain.from_iterable([x.correlations.keys() for x in executedBehaviours])))
        for behaviour in sorted(self._behaviours, key = lambda x: x.activation, reverse = True):
            rospy.loginfo("currently running behaviours: %s", executedBehaviours)
            rospy.loginfo("currently influenced sensors: %s", currentlyInfluencedSensors)
            ### now comes a series of tests that a behaviour must pass in order to get started ###
            if not behaviour.active: # it must be active
                continue
            if behaviour.isExecuting: # it must not already run
                continue
            if not behaviour.executable: # it must be executable
                continue
            if behaviour.activation < self._activationThreshold: # it must have high-enough activation
                continue
            interferingCorrelations = currentlyInfluencedSensors.intersection(set(behaviour.correlations.keys()))
            if len(interferingCorrelations) > 0: # it must not conflict with an already running behaviour ...
                alreadyRunningBehavioursRelatedToConflict = filter(lambda x: len(interferingCorrelations.intersection(set(x.correlations.keys()))) > 0, executedBehaviours)  # but it might be the case that the conflicting running behaviour(s) has/have less priority ...
                assert len(alreadyRunningBehavioursRelatedToConflict) <= 1 # This is true as long as there are no Aggregators (otherwise those behaviours must have been in conflict with each other). TODO: remove this assertion in case Aggregators are added
                # This implementation deals with a list although it it clear that there is at most one element.
                # With Aggregators this is not necessarily the case: There may be multiple behaviours running and affecting the same Aggregator, hence being correlated to the same sensor so they could all appear in this list.
                stoppableBehaviours = filter(lambda x: x.priority < behaviour.priority, alreadyRunningBehavioursRelatedToConflict) # TODO also take interruptability into consideration
                if set(stoppableBehaviours) == set(alreadyRunningBehavioursRelatedToConflict): # only continue if we can stop ALL offending behaviours. Otherwise we would kill some of them but that doesn't solve the problem and they died for nothing.
                    for conflictor in stoppableBehaviours:
                        rospy.loginfo("STOP BEHAVIOUR %s because it has less priority than %s", behaviour.name, conflictor.name)
                        conflictor.stop()
                        executedBehaviours.remove(conflictor) # remove it from the list of executed behaviours
                        currentlyInfluencedSensors.difference(set(conflictor.correlations.keys())) # remove all its correlations from the set of currently affected sensors
                    ### we have now made room for the higher-priority behaviour ###
                else:
                    continue
            ### if the behaviour got here it really is ready to be started ###
            rospy.loginfo("START BEHAVIOUR %s", behaviour.name)
            behaviour.start()
            rospy.loginfo("INCREASING ACTIVATION THRESHOLD TO %f", self._activationThreshold)
            self._activationThreshold *= (1/.8) # TODO: parameter for activationThresholdDecay
            currentlyInfluencedSensors.union(set(behaviour.correlations.keys()))
            executedBehaviours.append(behaviour)
        if len(executedBehaviours) == 0:
            self._activationThreshold *= .8     # TODO: parameter for activationThresholdDecay
            rospy.loginfo("REDUCING ACTIVATION THRESHOLD TO %f", self._activationThreshold)
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
    
    def __removeGoal(self, request):
        self._goals = filter(lambda x: x.name != request.name, self._goals) # kick out existing goals with that name. 
        return RemoveGoalResponse()
    
    def __removeBehaviour(self, request):
        self._behaviours = filter(lambda x: x.name != request.name, self._behaviours) # kick out existing behaviours with the same name.
        return RemoveBehaviourResponse()
    
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
    
    