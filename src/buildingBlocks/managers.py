'''
Created on 23.04.2015

@author: stephan
'''

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
        self._stepCounter = 0
    
    def step(self):
        print "STEP {0}".format(self._stepCounter)
        for sensor in self._sensors:
            print sensor, sensor.value
        print
        for goal in self._goals:
            print goal.name, "satisfaction", goal.statisfaction, "wishes", goal.getWishes()
        print
        for behaviour in self._behaviours:
            print behaviour.name
            print "\twishes", behaviour.getWishes()
            print "\tactivation from preconditions: ", behaviour.getActivationFromPreconditions()
            print "\tactivation from goals: ", behaviour.getActivationFromGoals()
            print "\tinhibition from goals: ", behaviour.getInhibitionFromGoals()
            print "\tactivation from predecessors: ", behaviour.getActivationFromPredecessors()
            print "\tactivation from successors: ", behaviour.getActivationFromSuccessors()
            print "\tinhibition from conflictors: ", behaviour.getInhibitionFromConflictors()
            print "\texecutable: {0} ({1})".format(behaviour.executable, behaviour.getPreconditionSatisfaction())
            behaviour.computeActivation()
            behaviour.commitActivation()
            print "\tactivation: ", behaviour.activation
        print
        executableBehaviours = [x for x in self._behaviours if x.executable or x.isExecuting] # make a list of executable or still executing behaviours
        for behaviour in executableBehaviours:
            if behaviour.activation >= self._activationThreshold or behaviour.isExecuting == True: # activate new behaviours or continue old ones
                print "RUNNING BEHAVIOUR ", behaviour.name
                behaviour.execute()

        print
        self._stepCounter += 1
    
    def addSensor(self, sensor):
        self._sensors.append(sensor)
        return sensor
    
    def addGoal(self, goal):
        self._goals.append(goal)
        return goal
    
    def addBehaviour(self, behaviour):
        behaviour.manager = self
        self._behaviours.append(behaviour)
        return behaviour
    
    @property
    def activationThreshold(self):
        return self._activationThreshold
    
    @activationThreshold.setter
    def activationThreshold(self, threshold):
        self._activationThreshold = threshold
        
    @property
    def sensors(self):
        return self._sensors
    
    @property
    def goals(self):
        return self._goals
    
    @property
    def behaviours(self):
        return self._behaviours