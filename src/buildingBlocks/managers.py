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
    
    def step(self):
        pass
    
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