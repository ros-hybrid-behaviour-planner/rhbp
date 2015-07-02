'''
Created on 23.04.2015

@author: stephan
'''

import rospy

class Manager(object):
    '''
    This is the manager class that keeps track of all elements in the network (behaviours, goals, sensors).
    Behaviours need this to know what sensors exist in the world and how they are correlated their measurement.
    They also need to know the goals to decide whether they are supporting them (which increases their activation) or inhibiting them (which decreases their activation).
    And last, they need to know whether another behaviour conflicts with them to reduce the activation of it.
    Also global constants like activation thresholds are stored here.
    '''

    def __init__(self):
        '''
        Constructor
        '''
        self._sensors = []
        self._goals = []
        self._behaviours = []
        self._activationThreshold = 10 # not sure how to set this just yet.
        self._activationDecay = .9     # not sure how to set this just yet.

        self._stepCounter = 0
    
    def step(self):
        rospy.loginfo("###################################### STEP {0} ######################################".format(self._stepCounter))
        rospy.loginfo("############ SENSOR STATI ############")
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
            rospy.loginfo("\tinhibition from conflictors: %s", behaviour.getInhibitionFromConflicted())
            rospy.loginfo("\texecutable: {0} ({1})".format(behaviour.executable, behaviour.getPreconditionSatisfaction()))
            behaviour.computeActivation()
        for behaviour in self._behaviours:
            rospy.loginfo("activation of %s after this step", behaviour.name)
            behaviour.commitActivation()
            rospy.loginfo("\tactivation: %f", behaviour.activation)
        rospy.loginfo("############## ACTIONS ###############")
        executableBehaviours = [x for x in self._behaviours if x.executable or x.isExecuting] # make a list of executable or still executing behaviours
        for behaviour in executableBehaviours:
            '''
            TODO: only run behaviours in parallel that do not interfere with each other (have no commom actuators) 
            '''
            if behaviour.activation >= self._activationThreshold or behaviour.isExecuting == True: # activate new behaviours or continue old ones
                rospy.loginfo("RUNNING BEHAVIOUR %s", behaviour.name)
                behaviour.execute()
        self._stepCounter += 1
    
    def addSensor(self, sensor):
        self._sensors.append(sensor)
        return sensor
    
    def addGoal(self, goal):
        self._goals.append(goal)
        return goal
    
    def addBehaviour(self, behaviour):
        behaviour.manager = self
        behaviour.activationDecay = self._activationDecay
        self._behaviours.append(behaviour)
        return behaviour
    
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