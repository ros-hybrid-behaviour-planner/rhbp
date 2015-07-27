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
        self._behaviours = []
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
        for goal in self._goals:
            ### collect information about behaviours ###
            goal.fetchStatus()
            rospy.loginfo("%s fulfillment: %f wishes %s", goal.name, goal.fulfillment, goal.wishes)
        self._goals = list(filter(lambda x: x.isPermanent or x.fulfillment < 1.0, self._goals)) # remove non-permanent goales that were achieved
            
        rospy.loginfo("########## BEHAVIOUR  STUFF ##########")
        for behaviour in self._behaviours:
            rospy.loginfo("%s", behaviour.name)
            rospy.loginfo("\twishes %s", behaviour.wishes)
            rospy.loginfo("\tactivation from preconditions: %s", behaviour.activationFromPreconditions)
            rospy.loginfo("\tactivation from goals: %s", behaviour.getActivationFromGoals())
            rospy.loginfo("\tinhibition from goals: %s", behaviour.getInhibitionFromGoals())
            rospy.loginfo("\tactivation from predecessors: %s", behaviour.getActivationFromPredecessors())
            rospy.loginfo("\tactivation from successors: %s", behaviour.getActivationFromSuccessors())
            rospy.loginfo("\tinhibition from conflicted: %s", behaviour.getInhibitionFromConflicted())
            rospy.loginfo("\texecutable: {0} ({1})".format(behaviour.executable, behaviour.preconditionSatisfaction))
            behaviour.computeActivation()
        for behaviour in self._behaviours:
            rospy.loginfo("activation of %s after this step", behaviour.name)
            behaviour.commitActivation()
            rospy.loginfo("\tactivation: %f", behaviour.activation)
        rospy.loginfo("############## ACTIONS ###############")
        executableBehaviours = [x for x in self._behaviours if (x.executable and x.activation >= self._activationThreshold) and not x.isExecuting] # make a list of executable  behaviours
        executedBehaviours = list(filter(lambda x: x.isExecuting, self._behaviours))
        currentlyInfluencedSensors = set(list(itertools.chain.from_iterable([x.correlations.keys() for x in executedBehaviours])))
        rospy.loginfo("currentlyInfluencedSensors: %s", currentlyInfluencedSensors)
        executableBehaviours = list(filter(lambda x: len(currentlyInfluencedSensors.intersection(set(x.correlations.keys()))) == 0, executableBehaviours))
        if len(executableBehaviours) > 0:
            for behaviour in executableBehaviours:
                rospy.loginfo("START BEHAVIOUR %s", behaviour.name)
                behaviour.start()
            self._activationThreshold *= (1/.8)
            rospy.loginfo("INCREASING ACTIVATION THRESHOLD TO %f", self._activationThreshold)
        else:
            self._activationThreshold *= .8
            rospy.loginfo("REDUCING ACTIVATION THRESHOLD TO %f", self._activationThreshold)
        self._stepCounter += 1
    
    def __addGoal(self, request):
        self._goals = list(filter(lambda x: x.name != request.name, self._goals)) # kick out existing goals with the same name. 
        goal = Goal(request.name, request.permanent)
        self._goals.append(goal)
        rospy.loginfo("A goal with name %s registered", goal.name)
        return AddGoalResponse()
    
    def __addBehaviour(self, request):
        self._behaviours = list(filter(lambda x: x.name != request.name, self._behaviours)) # kick out existing behaviours with the same name.
        behaviour = Behaviour(request.name)
        behaviour.manager = self
        behaviour.activationDecay = self._activationDecay
        self._behaviours.append(behaviour)
        rospy.loginfo("A behaviour with name %s registered", behaviour.name)
        return AddBehaviourResponse()
    
    def __removeGoal(self, request):
        self._goals = list(filter(lambda x: x.name != request.name, self._goals)) # kick out existing goals with that name. 
        return RemoveGoalResponse()
    
    def __removeBehaviour(self, request):
        self._behaviours = list(filter(lambda x: x.name != request.name, self._behaviours)) # kick out existing behaviours with the same name.
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