'''
Created on 13.04.2015

@author: stephan
''' 
from __future__ import division # force floating point division when using plain /
import rospy
import operator
import conditions
import warnings
import itertools


class Behaviour(object):
    '''
    This is the smallest entity of an action.
    It may take a while or finish immediately.
    '''
    
    _instanceCounter = 0 # static counter to get distinguishable names

    def __init__(self, name = None, **kwargs):
        '''
        Constructor
        '''
        self._name = name if name else "Behaviour {0}".format(Behaviour._instanceCounter)
        self._preconditions = []
        self._isExecuting = False  # Set this to True if this behaviour is selected for execution.
        self._readyThreshold = kwargs["readyThreshold"] if "readyThreshold" in kwargs else 0.8 # This is the threshold that the preconditions must reach in order for this behaviour to be executable.
        self._correlations = kwargs["correlations"] if "correlations" in kwargs else {}        # Stores sensor correlations in dict in form: sensor <Sensor> : correlation <float> [-1 to 1]. 1 Means high positive correlation to the value or makes it become True, -1 the opposite and 0 does not affect anything.
        self._predecessors = []    # List of predecessors of this behaviour. This list is refreshed each iteration based on correlations.
        self._successors = []      # List of successors. This list is refreshed each iteration based on correlations.
        self._conflictors = []     # List of successors. This list is refreshed each iteration based on correlations.
        self._manager = None       # This is the Manager that supplies global variables an access to all foreign objects
        self._activation = 0.0     # This is the magic activation that it's all about
        self._activationDecay = .9 # This reduces accumulated activation if the situation does not fit any more
        self.__currentActivationStep = 0.0
        Behaviour._instanceCounter += 1
        self.__logFile = open(self._name, 'w')
        self.__logFile.write('{0}\n'.format(self._name))
    
    def __del__(self):
        '''
        Destructor
        '''
        self.__logFile.close()
        
    def addPrecondition(self, precondition):
        '''
        This method adds an precondition to the behaviour.
        There is an AND relationship between all elenents (all have to be fulfilled so that the behaviour is ready)
        To enable OR behaviour use the pseudo Conditional Disjunction.
        '''
        if issubclass(type(precondition), conditions.Conditonal):
            self._preconditions.append(precondition)
        else:
            warnings.warn("That's no conditional object!")
    
    def getPreconditionSatisfaction(self):
        '''
        This method should return the overall activation from all preconditions.
        In the easiest case this is equivalent to the product of the individual activations.
        '''
        return reduce(operator.mul, (x.satisfaction for x in self._preconditions), 1)
    
    def getWishes(self):
        '''
        This method returns a dict of wishes.
        For all sensors wrapped in conditions for this goal this dict says what changes are necessary to achieve it
        '''
        return dict(itertools.chain.from_iterable([x.getWishes() for x in self._preconditions]))

    def getActivationFromPreconditions(self):
        '''
        This method computes the activation from the situation.
        It is the average satisfaction of preconditions.
        '''
        return reduce(lambda x, y: x + y, (x.satisfaction for x in self._preconditions)) / len(self._preconditions)
    
    def getActivationFromGoals(self):
        '''
        This method computes the activation from goals.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        '''
        activatedByGoals = []
        for goal in self._manager.goals:
            for (sensor, indicator) in goal.getWishes().iteritems():
                if sensor in self._correlations.keys() and self._correlations[sensor] * indicator > 0: # This means we affect the sensor in a way that is desirable by the goal
                    numBehavioursActivatedBySameGoal = len([b for b in self._manager.behaviours if sensor in b.correlations.keys() and b.correlations[sensor] * indicator > 0.0]) # the variable name says it all
                    rospy.logdebug("Calculating activation from goals for %s. There is/are %d behaviour(s) that supports %s via %s: %s",  self.name, numBehavioursActivatedBySameGoal, goal.name, sensor, [b for b in self._manager.behaviours if sensor in b.correlations.keys() and b.correlations[sensor] * indicator > 0.0])
                    activatedByGoals.append((goal, self._correlations[sensor] * indicator / numBehavioursActivatedBySameGoal))            # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
        return (0.0,) if len(activatedByGoals) == 0 else (reduce(lambda x, y: x + y, (x[1] for x in activatedByGoals)), activatedByGoals)
    
    def getInhibitionFromGoals(self):
        '''
        This method computes the inhibition (actually, activation but negative sign!) caused by goals it conflicts with.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        '''
        inhibitedByGoals = []
        for goal in self._manager.goals:
            for (sensor, indicator) in goal.getWishes().iteritems():
                if sensor in self._correlations.keys() and self._correlations[sensor] * indicator < 0: # This means we affect the sensor in a way that is not desirable by the goal
                    numBehavioursInhibitedBySameGoal = len([b for b in self._manager.behaviours if sensor in b.correlations.keys() and b.correlations[sensor] * indicator < 0.0]) # the variable name says it all
                    rospy.logdebug("Calculating inhibition from goals for %s. There is/are %d behaviour(s) that contradict %s via %s: %s",  self.name, numBehavioursInhibitedBySameGoal, goal.name, sensor, [b for b in self._manager.behaviours if sensor in b.correlations.keys() and b.correlations[sensor] * indicator < 0.0])
                    inhibitedByGoals.append((goal, self._correlations[sensor] * indicator / numBehavioursInhibitedBySameGoal))            # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Note that this is negative, hence the name inhibition! Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
        return (0.0,) if len(inhibitedByGoals) == 0 else (reduce(lambda x, y: x + y, (x[1] for x in inhibitedByGoals)), inhibitedByGoals)
    
    def getActivationFromPredecessors(self):
        '''
        This method computes the activation based on the fact that other behaviours can fulfill a precondition of this behaviour.
        This is scaled by the "readyness" (precondition satisfaction) of the predecessor as it makes only sense to activate a successor if it is likely that it is executable soon.
        '''
        activatedByPredecessors = []
        for behaviour in self._manager.behaviours:
            if behaviour == self or not behaviour.executable: # ignore ourselves and non-executable behaviours
                continue
            for (sensor, indicator) in self.getWishes().iteritems():
                if sensor in behaviour.correlations.keys() and behaviour.correlations[sensor] *  behaviour.getPreconditionSatisfaction() * indicator > 0.0:  # If a predecessor can satisfy my precondition
                    numSuccessorsOfThatPredecessorAffectedByThisCorrelation = len([b for b in self._manager.behaviours if sensor in b.getWishes().keys() and b.getWishes()[sensor] * indicator > 0.0]) # the variable name says it all
                    rospy.logdebug("Calculating activation from predecessors for %s. There is/are %d successor(s) of %s via %s: %s",  self.name, numSuccessorsOfThatPredecessorAffectedByThisCorrelation, behaviour.name, sensor, [b for b in self._manager.behaviours if sensor in b.getWishes().keys() and b.getWishes()[sensor] * indicator > 0.0])
                    activatedByPredecessors.append((behaviour, sensor, behaviour.correlations[sensor] * indicator * behaviour.getPreconditionSatisfaction() * behaviour.activation / numSuccessorsOfThatPredecessorAffectedByThisCorrelation)) # The activation we get is the likeliness that our predecessor fulfills the preconditions soon. numSuccessorsOfThatPredecessorAffectedByThisCorrelation knows how many more behaviours will get activation from this predecessor so we distribute it equally
        return (0.0,) if len(activatedByPredecessors) == 0 else (reduce(lambda x, y: x + y, (x[2] for x in activatedByPredecessors)), activatedByPredecessors)

    def getActivationFromSuccessors(self):
        '''
        This method computes the activation this behaviour receives because it fulfills a precondition (is a predecessor) of a successor which has unfulfilled wishes.
        '''
        activatedBySuccessors = []
        for behaviour in self._manager.behaviours:
            if behaviour == self: # ignore ourselves
                continue
            behaviourWishes = behaviour.getWishes()                    # this is what the behaviour wants
            for (sensor, indicator) in self._correlations.iteritems(): # this is what we do to sensors
                if sensor in behaviourWishes.keys():                   # if we affect it somehow
                    if behaviourWishes[sensor] * indicator > 0:        # we are a predecessor so we get activation from that successor
                        numPredecessorsOfThatSuccessorAffectedByThisCorrelation = len([b for b in self._manager.behaviours if sensor in b.correlations.keys() and b.correlations[sensor] * indicator > 0.0]) # the variable name says it all
                        rospy.logdebug("Calculating activation from successors for %s. There is/are %d predecessor(s) of %s via %s: %s", self.name, numPredecessorsOfThatSuccessorAffectedByThisCorrelation, behaviour.name, sensor, [b for b in self._manager.behaviours if sensor in b.correlations.keys() and b.correlations[sensor] * indicator > 0.0])
                        activatedBySuccessors.append((behaviour, sensor, behaviourWishes[sensor] * indicator * behaviour.activation / numPredecessorsOfThatSuccessorAffectedByThisCorrelation)) # The activation we get is our expected contribution to the fulfillment of our successors precondition. Actually only the value is needed but it is a tuple for debug purposes. numPredecessorsOfThatSuccessorAffectedByThisCorrelation is used to distribute activation among all predecessors
        return (0.0,) if len(activatedBySuccessors) == 0 else (reduce(lambda x, y: x + y, (x[2] for x in activatedBySuccessors)), activatedBySuccessors)

    def getInhibitionFromConflicted(self):
        '''
        This method computes the inhibition (actually, activation but negative sign!) caused by other behaviours whose preconditions get antagonized when this behaviour runs.
        '''
        inhibitionFromConflictors = []
        for behaviour in self._manager.behaviours:
            if behaviour == self: # ignore ourselves
                continue
            behaviourWishes = behaviour.getWishes()                    # this is what the behaviour wants
            for (sensor, indicator) in self._correlations.iteritems(): # this is what we do to sensors
                if sensor in behaviourWishes.keys():                   # if we affect it somehow
                    if behaviourWishes[sensor] * indicator < 0:        # we are a conflictor so we get inhibited
                        numBehavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation = len([b for b in self._manager.behaviours if sensor in b.correlations.keys() and b.correlations[sensor] * behaviour.getWishes()[sensor] < 0.0]) # the variable name says it all
                        rospy.logdebug("Calculating inhibition from conflicted for %s. %s is conflicted via %s by %d behaviour(s): %s", self.name, behaviour.name, sensor, numBehavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation, [b for b in self._manager.behaviours if sensor in b.correlations.keys() and b.correlations[sensor] * behaviour.getWishes()[sensor] < 0.0])
                        inhibitionFromConflictors.append((behaviour, sensor, behaviourWishes[sensor] * indicator * behaviour.activation / numBehavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation)) # The inhibition experienced is my bad influence (my correlation to this sensor) times the wish of the other behaviour concerning this sensor. Actually only the value is needed but it is a tuple for debug purposes. numBehavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation knows how many more behaviours cause the same conflict to this conflictor so its inhibition shall be distributed among them.
        return (0.0,) if len(inhibitionFromConflictors) == 0 else (reduce(lambda x, y: x + y, (x[2] for x in inhibitionFromConflictors)), inhibitionFromConflictors)

    def computeActivation(self):
        '''
        This method sums up all components of activation to compute the additional activation in this step.
        '''
        self.__currentActivationStep = self.getActivationFromPreconditions() \
            + self.getActivationFromGoals()[0] \
            + self.getInhibitionFromGoals()[0] \
            + self.getActivationFromPredecessors()[0] \
            + self.getActivationFromSuccessors()[0] \
            + self.getInhibitionFromConflicted()[0]
    
    def commitActivation(self):
        '''
        This method applies the activatipn of this iteration to the overall activation.
        '''
        self._activation = self._activation * self._activationDecay + self.__currentActivationStep
        if self._activation < 0.0:
            self._activation = 0
        self.__currentActivationStep = 0.0        
        self.__logFile.write("{0}\n".format(self._activation))
    
    def execute(self):
        '''
        This method is a wrapper around the behaviours actual action.
        It sets internal parameter like _isExecuting and resets the activation once the behaviour is done (returns False)
        TODO: multi thread this so that behaviour network can continue to work on other things (actionlib?!)
        '''
        self._isExecuting = True
        if self.action() == False: # The action has finished
            self._isExecuting = False
            self._activation = 0.0
            
        
    def action(self):
        return False
    
    @property
    def correlations(self):
        return self._correlations
    
    @correlations.setter
    def correlations(self, correlations):
        '''
        This is for initializing the correlations.
        correlations must be a dict of form sensor <Sensor> : correlation <float> [-1 to 1].
        '''
        self._correlations = correlations
    
    @property
    def activation(self):
        return self._activation
    
    @property
    def readyThreshold(self):
        return self._readyThreshold
    
    @readyThreshold.setter
    def readyThreshold(self, threshold):
        self._readyThreshold = float(threshold)
    
    @property
    def activationDecay(self):
        return self._activationDecay
    
    @activationDecay.setter
    def activationDecay(self, rate):
        self._activationDecay = rate
        
    @property
    def manager(self):
        return self._manager
    
    @manager.setter
    def manager(self, manager):
        self._manager = manager
    
    @property
    def preconditions(self):
        return self._preconditions
        
    @property
    def executable(self):
        return self.getPreconditionSatisfaction() >= self._readyThreshold
    
    @property
    def name(self):
        return self._name
    
    @name.setter
    def name(self, newName):
        self._name = newName
        
    @property
    def isExecuting(self):
        return self._isExecuting
    
    def __str__(self):
        return "{0} with the following preconditions:\n{1}".format(self._name, "\n".join([str(x) for x in self._preconditions]))
    
    def __repr__(self):
        return self._name
        