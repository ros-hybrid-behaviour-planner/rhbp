'''
Created on 23.04.2015

@author: stephan
'''

import rospy
from behaviourPlannerPython.srv import AddBehaviour, AddBehaviourResponse
from buildingBlocks.behaviour import Behaviour
import behaviour


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
        self.addBehaviourService = rospy.Service('AddBehaviour', AddBehaviour, self.addBehaviour)
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
        self.addBehaviourService.shutdown()
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
            rospy.loginfo("%s satisfaction: %f wishes %s", goal.name, goal.statisfaction, goal.getWishes())
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
        executableBehaviours = [x for x in self._behaviours if (x.executable and x.activation >= self._activationThreshold) or x.isExecuting] # make a list of executable or still executing behaviours
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
    
    
    def addGoal(self, goal):
        self._goals.append(goal)
        return goal
    
    def addBehaviour(self, request):
        # TODO: check if already existing and kick out or do nothing
        behaviour = Behaviour(request.name)
        behaviour.manager = self
        behaviour.activationDecay = self._activationDecay
        self._behaviours.append(behaviour)
        rospy.loginfo("A behaviour with name %s registered", behaviour.name)
        return AddBehaviourResponse()
    
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