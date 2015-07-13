'''
Created on 23.04.2015

@author: stephan
'''

import rospy
from behaviourPlannerPython.srv import *
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
        rospy.Service('addBehaviour', addBehaviour, self.addBehaviour)
        self._sensors = []
        self._goals = []
        self._behaviours = []
        self._activationThreshold = 10 # not sure how to set this just yet.
        self._activationDecay = .9     # not sure how to set this just yet.
        self._activationThreshold = kwargs["activationThreshold"] if "activationThreshold" in kwargs else 7.0 # not sure how to set this just yet.
        self._activationDecay = kwargs["activationDecay"] if "activationDecay" in kwargs else .9  # not sure how to set this just yet.
        self._stepCounter = 0
        self.__logFile = open("sensors.log", 'w')
        self.__threshFile = open("threshold.log", 'w')
        
    def __del__(self):
        self.__logFile.close()
        self.__threshFile.close()
    
    def step(self):
        if self._stepCounter == 0:
            self.__logFile.write("{0}\n".format("\t".join([str(s) for s in self._sensors])))
            self.__threshFile.write("{0}\n".format("activationThreshold"))
            
        rospy.loginfo("###################################### STEP {0} ######################################".format(self._stepCounter))
        for behaviour in self._behaviours:
            rospy.loginfo("Planner manager set() waiting for service %s", behaviour.name + 'getActivation')
            rospy.wait_for_service(behaviour.name + 'getActivation')
            try:
                getActivationRequest = rospy.ServiceProxy(behaviour.name + 'getActivation', getActivation)
                activation = getActivationRequest()
                rospy.loginfo("Behaviour %s report activation of %f", behaviour.name, activation.activation)
            except rospy.ServiceException as e:
                rospy.logerr("ROS service exception in Manager step(): %s", e)
                
            rospy.loginfo("Planner manager set() waiting for service %s", behaviour.name + 'getWishes')
            rospy.wait_for_service(behaviour.name + 'getWishes')
            try:
                getWishesRequest = rospy.ServiceProxy(behaviour.name + 'getWishes', getWishes)
                wishes = getWishesRequest()
                rospy.loginfo("Behaviour %s reports the following wishes: %s", behaviour.name, wishes.wishes)
            except rospy.ServiceException as e:
                rospy.logerr("ROS service exception in Manager step(): %s", e)
                
        """
        rospy.loginfo("############ SENSOR STATI ############")
        self.__logFile.write("{0}\n".format("\t".join([str(float(s.value)) for s in self._sensors])))
        self.__threshFile.write("{0}\n".format(self._activationThreshold))
        for sensor in self._sensors:
            rospy.loginfo("%s %s", sensor, sensor.value)
        rospy.loginfo("############# GOAL STATI #############")
        for goal in self._goals:
            rospy.loginfo("%s satisfaction: %f wishes %s", goal.name, goal.statisfaction, goal.getWishes())
        rospy.loginfo("########## BEHAVIOUR  STUFF ##########")
        for behaviour in self._behaviours:
            rospy.loginfo("%s", behaviour.name)
            rospy.loginfo("\twishes %s", behaviour.getWishes())
            rospy.loginfo("\tactivation from preconditions: %s", behaviour.getActivationFromPreconditions())
            rospy.loginfo("\tactivation from goals: %s", behaviour.getActivationFromGoals())
            rospy.loginfo("\tinhibition from goals: %s", behaviour.getInhibitionFromGoals())
            rospy.loginfo("\tactivation from predecessors: %s", behaviour.getActivationFromPredecessors())
            rospy.loginfo("\tactivation from successors: %s", behaviour.getActivationFromSuccessors())
            rospy.loginfo("\tinhibition from conflicted: %s", behaviour.getInhibitionFromConflicted())
            rospy.loginfo("\texecutable: {0} ({1})".format(behaviour.executable, behaviour.getPreconditionSatisfaction()))
            behaviour.computeActivation()
        for behaviour in self._behaviours:
            rospy.loginfo("activation of %s after this step", behaviour.name)
            behaviour.commitActivation()
            rospy.loginfo("\tactivation: %f", behaviour.activation)
        rospy.loginfo("############## ACTIONS ###############")
        executableBehaviours = [x for x in self._behaviours if (x.executable and x.activation >= self._activationThreshold) or x.isExecuting] # make a list of executable or still executing behaviours
        if len(executableBehaviours) > 0:
            for behaviour in executableBehaviours:
                '''
                TODO: only run behaviours in parallel that do not interfere with each other (have no common actuators) 
                '''
                rospy.loginfo("RUNNING BEHAVIOUR %s", behaviour.name)
                behaviour.execute()
            self._activationThreshold *= (1/.8)
            rospy.loginfo("INCREASING ACTIVATION THRESHOLD TO %f", self._activationThreshold)
        else:
            self._activationThreshold *= .8
            rospy.loginfo("REDUCING ACTIVATION THRESHOLD TO %f", self._activationThreshold)
        """
        self._stepCounter += 1
    
    def addSensor(self, sensor):
        self._sensors.append(sensor)
        return sensor
    
    def addGoal(self, goal):
        self._goals.append(goal)
        return goal
    
    def addBehaviour(self, request):
        behaviour = Behaviour(request.name)
        behaviour.manager = self
        behaviour.activationDecay = self._activationDecay
        self._behaviours.append(behaviour)
        rospy.loginfo("A behaviour with name %s registered", behaviour.name)
        return addBehaviourResponse()
    
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
    def sensors(self):
        return self._sensors
    
    @property
    def goals(self):
        return self._goals
    
    @property
    def behaviours(self):
        return self._behaviours