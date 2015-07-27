'''
Created on 13.04.2015

@author: stephan
''' 
from __future__ import division # force floating point division when using plain /
import rospy
import conditions
import operator
import warnings
import itertools
from std_srvs.srv import Empty, EmptyResponse
from behaviourPlannerPython.srv import AddBehaviour, GetStatus, GetStatusResponse
from behaviourPlannerPython.msg import Wish, Correlation

class Behaviour(object):
    '''
    This is the internal representation of a behaviour node
    '''
    
    _instanceCounter = 0 # static counter to get distinguishable names

    def __init__(self, name):
        '''
        Constructor
        '''
        self._name = name if name else "Behaviour {0}".format(Behaviour._instanceCounter)
        self._isExecuting = False   # Set this to True if this behaviour is selected for execution.
        self._correlations = {}     # Stores sensor correlations in dict in form: {sensor name <string> : correlation <float> [-1 to 1]}. 1 Means high positive correlation to the value or makes it become True, -1 the opposite and 0 does not affect anything. We get this value via getStatus service of actual behaviour node
        self._wishes = {}           # Stores wishes exactly like correlations. We get this via getStatus service of actual behaviour node
        self._manager = None        # This is the Manager that supplies global variables an access to all foreign objects
        self._activation = 0.0      # This is the magic activation that it's all about
        self._activationFromPreconditions = 0.0 # We get it via getStatus service of actual behaviour node
        self._preconditionSatisfaction = 0.0    # We get it via getStatus service of actual behaviour node
        self._readyThreshold = 0.0  # This is the threshold that the preconditionSatisfaction must reach in order for this behaviour to be executable. We get this value via getStatus service of actual behaviour node.
        self._activationDecay = 0.0 # This reduces accumulated activation if the situation does not fit any more
        self._active = True          # This indicates (if True) that there have been no severe issues in the actual behaviour node and the behaviour can be expected to be operational. If the actual behaviour reports active == False we will ignore it in activation computation.
        self.__currentActivationStep = 0.0
        Behaviour._instanceCounter += 1
        self.__logFile = open("{0}.log".format(self._name), 'w')
        self.__logFile.write('Time\t{0}\n'.format(self._name))
    
    def __del__(self):
        '''
        Destructor
        '''
        self.__logFile.close()
    
    def fetchStatus(self):
        '''
        This method fetches the status from the actual behaviour node via GetStatus service call
        '''
        rospy.logdebug("Waiting for service %s", self._name + 'GetStatus')
        rospy.wait_for_service(self._name + 'GetStatus')
        try:
            getStatusRequest = rospy.ServiceProxy(self._name + 'GetStatus', GetStatus)
            status = getStatusRequest()
            self._activationFromPreconditions = status.activation
            self._correlations = dict([(correlation.sensorName, correlation.indicator) for correlation in status.correlations])
            self._preconditionSatisfaction = status.satisfaction
            self._readyThreshold = status.threshold
            self._wishes = dict([(wish.sensorName, wish.indicator) for wish in status.wishes])
            if self._isExecuting == True and status.isExecuting == False:
                rospy.loginfo("%s finished. resetting activation", self._name)
                self._activation = 0.0
            self._isExecuting = status.isExecuting
            self._active = status.active
            rospy.logdebug("%s reports the following status:\nactivation %s\ncorrelations %s\nprecondition satisfaction %s\n ready threshold %s\nwishes %s", self._name, self._activationFromPreconditions, self._correlations, self._preconditionSatisfaction, self._readyThreshold, self._wishes)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in GetStatus of %s: %s", self._name, e)    
            
    def getActivationFromGoals(self):
        '''
        This method computes the activation from goals.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        '''
        activatedByGoals = []
        for goal in self._manager.activeGoals:
            for (sensorName, indicator) in goal.wishes.iteritems():
                if sensorName in self._correlations.keys() and self._correlations[sensorName] * indicator > 0: # This means we affect the sensor in a way that is desirable by the goal
                    numBehavioursActivatedBySameGoal = len([b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] * indicator > 0.0]) # the variable name says it all
                    rospy.logdebug("Calculating activation from goals for %s. There is/are %d active behaviour(s) that supports %s via %s: %s",  self.name, numBehavioursActivatedBySameGoal, goal.name, sensorName, [b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] * indicator > 0.0])
                    activatedByGoals.append((goal, self._correlations[sensorName] * indicator / numBehavioursActivatedBySameGoal))            # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
        return (0.0,) if len(activatedByGoals) == 0 else (reduce(lambda x, y: x + y, (x[1] for x in activatedByGoals)), activatedByGoals)
    
    def getInhibitionFromGoals(self):
        '''
        This method computes the inhibition (actually, activation but negative sign!) caused by goals it conflicts with.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        '''
        inhibitedByGoals = []
        for goal in self._manager.activeGoals:
            for (sensorName, indicator) in goal.wishes.iteritems():
                if sensorName in self._correlations.keys() and self._correlations[sensorName] * indicator < 0: # This means we affect the sensor in a way that is not desirable by the goal
                    numBehavioursInhibitedBySameGoal = len([b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] * indicator < 0.0]) # the variable name says it all
                    rospy.logdebug("Calculating inhibition from goals for %s. There is/are %d behaviour(s) that contradict %s via %s: %s",  self.name, numBehavioursInhibitedBySameGoal, goal.name, sensorName, [b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] * indicator < 0.0])
                    inhibitedByGoals.append((goal, self._correlations[sensorName] * indicator / numBehavioursInhibitedBySameGoal))            # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Note that this is negative, hence the name inhibition! Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
        return (0.0,) if len(inhibitedByGoals) == 0 else (reduce(lambda x, y: x + y, (x[1] for x in inhibitedByGoals)), inhibitedByGoals)
    
    def getActivationFromPredecessors(self):
        '''
        This method computes the activation based on the fact that other behaviours can fulfill a precondition of this behaviour.
        This is scaled by the "readyness" (precondition satisfaction) of the predecessor as it makes only sense to activate a successor if it is likely that it is executable soon.
        '''
        activatedByPredecessors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == self or not behaviour.executable: # ignore ourselves and non-executable behaviours
                continue
            for (sensorName, indicator) in self._wishes.iteritems():
                if sensorName in behaviour.correlations.keys() and behaviour.correlations[sensorName] *  behaviour.preconditionSatisfaction * indicator > 0.0:  # If a predecessor can satisfy my precondition
                    numSuccessorsOfThatPredecessorAffectedByThisCorrelation = len([b for b in self._manager.activeBehaviours if sensorName in b.wishes.keys() and b.wishes[sensorName] * indicator > 0.0]) # the variable name says it all
                    rospy.logdebug("Calculating activation from predecessors for %s. There is/are %d active successor(s) of %s via %s: %s",  self._name, numSuccessorsOfThatPredecessorAffectedByThisCorrelation, behaviour.name, sensorName, [b for b in self._manager.activeBehaviours if sensorName in b.wishes.keys() and b.wishes[sensorName] * indicator > 0.0])
                    activatedByPredecessors.append((behaviour, sensorName, behaviour.correlations[sensorName] * indicator * behaviour.preconditionSatisfaction / numSuccessorsOfThatPredecessorAffectedByThisCorrelation)) # The activation we get is the likeliness that our predecessor fulfills the preconditions soon. numSuccessorsOfThatPredecessorAffectedByThisCorrelation knows how many more behaviours will get activation from this predecessor so we distribute it equally
        return (0.0,) if len(activatedByPredecessors) == 0 else (reduce(lambda x, y: x + y, (x[2] for x in activatedByPredecessors)), activatedByPredecessors)

    def getActivationFromSuccessors(self):
        '''
        This method computes the activation this behaviour receives because it fulfills a precondition (is a predecessor) of a successor which has unfulfilled wishes.
        '''
        activatedBySuccessors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == self: # ignore ourselves
                continue
            for (sensorName, indicator) in self._correlations.iteritems(): # this is what we do to sensors
                if sensorName in behaviour.wishes.keys():                  # if we affect other behaviours wishes somehow
                    if behaviour.wishes[sensorName] * indicator > 0:       # if we are a predecessor so we get activation from that successor
                        numPredecessorsOfThatSuccessorAffectedByThisCorrelation = len([b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] * indicator > 0.0]) # the variable name says it all
                        rospy.logdebug("Calculating activation from successors for %s. There is/are %d active predecessor(s) of %s via %s: %s", self._name, numPredecessorsOfThatSuccessorAffectedByThisCorrelation, behaviour.name, sensorName, [b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] * indicator > 0.0])
                        activatedBySuccessors.append((behaviour, sensorName, behaviour.wishes[sensorName] * indicator / numPredecessorsOfThatSuccessorAffectedByThisCorrelation)) # The activation we get is our expected contribution to the fulfillment of our successors precondition. Actually only the value is needed but it is a tuple for debug purposes. numPredecessorsOfThatSuccessorAffectedByThisCorrelation is used to distribute activation among all predecessors
        return (0.0,) if len(activatedBySuccessors) == 0 else (reduce(lambda x, y: x + y, (x[2] for x in activatedBySuccessors)), activatedBySuccessors)

    def getInhibitionFromConflicted(self):
        '''
        This method computes the inhibition (actually, activation but negative sign!) caused by other behaviours whose preconditions get antagonized when this behaviour runs.
        It works different from Maes and others: It does not account for behaviours that will make things bad if they are currently OK. It accounts only for things that are not OK and works against behaviours that would make it even worse.
        '''
        inhibitionFromConflictors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == self: # ignore ourselves
                continue
            for (sensorName, indicator) in self._correlations.iteritems(): # this is what we do to sensors
                if sensorName in behaviour.wishes.keys():                  # if we affect other behaviours somehow
                    if behaviour.wishes[sensorName] * indicator < 0:       # if we are a conflictor, we get inhibited
                        numBehavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation = len([b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] * behaviour.wishes[sensorName] < 0.0]) # the variable name says it all
                        rospy.logdebug("Calculating inhibition from conflicted for %s. %s is conflicted via %s by %d behaviour(s): %s", self.name, behaviour.name, sensorName, numBehavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation, [b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] * behaviour.wishes[sensorName] < 0.0])
                        inhibitionFromConflictors.append((behaviour, sensorName, behaviour.wishes[sensorName] * indicator / numBehavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation)) # The inhibition experienced is my bad influence (my correlation to this sensorName) times the wish of the other behaviour concerning this sensorName. Actually only the value is needed but it is a tuple for debug purposes. numBehavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation knows how many more behaviours cause the same conflict to this conflictor so its inhibition shall be distributed among them.
        return (0.0,) if len(inhibitionFromConflictors) == 0 else (reduce(lambda x, y: x + y, (x[2] for x in inhibitionFromConflictors)), inhibitionFromConflictors)

    def computeActivation(self):
        '''
        This method sums up all components of activation to compute the additional activation in this step.
        '''
        self.__currentActivationStep = self._activationFromPreconditions \
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
        self.__logFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._activation))
        self.__logFile.flush()
    
    def start(self):
        '''
        TODO: communicate non-blocking with actual behaviour node (actionlib?!)
        '''
        self._isExecuting = True
        assert self._active
        try:
            rospy.logdebug("Waiting for service %s", self._name + 'Start')
            rospy.wait_for_service(self._name + 'Start')
            startRequest = rospy.ServiceProxy(self._name + 'Start', Empty)
            startRequest()
            rospy.loginfo("Started action of %s", self._name)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception while calling %s: %s", self._name + 'Start', e)
            
    @property
    def wishes(self):
        return self._wishes
    
    @property
    def correlations(self):
        return self._correlations
    
    @property
    def activation(self):
        return self._activation
    
    @property
    def readyThreshold(self):
        return self._readyThreshold
    
    @property
    def preconditionSatisfaction(self):
        return self._preconditionSatisfaction
    
    @property
    def activationFromPreconditions(self):
        return self._activationFromPreconditions
    
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
    def executable(self):
        return self._preconditionSatisfaction >= self._readyThreshold
    
    @property
    def name(self):
        return self._name
    
    @property
    def active(self):
        return self._active
        
    @property
    def isExecuting(self):
        return self._isExecuting
    
    def __str__(self):
        return self._name
    
    def __repr__(self):
        return self._name



class BehaviourBase(object):
    '''
    This is the base class for behaviour nodes in python
    '''

    def __init__(self, name, **kwargs):
        '''
        Constructor
        '''
        self._name = name # a unique name is mandatory
        self._getActivationService = rospy.Service(self._name + 'GetStatus', GetStatus, self.getStatus)
        self._startService = rospy.Service(self._name + 'Start', Empty, self.startCallback)
        self._preconditions = kwargs["preconditions"] if "preconditions" in kwargs else [] # This are the preconditions for the behaviour. They may not be used but the default implementations of computeActivation(), computeSatisfaction(), and computeWishes work them. See addPrecondition()
        self._isExecuting = False  # Set this to True if this behaviour is selected for execution.
        self._correlations = kwargs["correlations"] if "correlations" in kwargs else {} # Stores sensor correlations in dict in form: {sensor name <string> : correlation <float> [-1 to 1]}. 1 Means high positive correlation to the value or makes it become True, -1 the opposite and 0 does not affect anything. # be careful with the sensor Name! It has to actually match something that exists!
        self._readyThreshold = kwargs["readyThreshold"] if "readyThreshold" in kwargs else 0.8 # This is the threshold that the preconditions must reach in order for this behaviour to be executable.
        self._plannerPrefix = kwargs["plannerPrefix"] if "plannerPrefix" in kwargs else "" # if you have multiple planners in the same ROS environment use a prefix to name the right one.
        self._active = True # if anything in the behaviour is not initialized or working properly this must be set to False and communicated via getStatus service
        try:
            rospy.loginfo("BehaviourBase constructor waiting for registration at planner manager with prefix '%s' for behaviour node %s", self._plannerPrefix, self._name)
            rospy.wait_for_service(self._plannerPrefix + 'AddBehaviour')
            registerMe = rospy.ServiceProxy(self._plannerPrefix + 'AddBehaviour', AddBehaviour)
            registerMe(self._name)
            rospy.loginfo("BehaviourBase constructor registered at planner manager with prefix '%s' for behaviour node %s", self._plannerPrefix, self._name)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in BehaviourBase constructor (for behaviour node %s): %s", self._name, e)
    
    def __del__(self):
        '''
        Destructor
        '''
        try:
            self._getStatusService.shutdown()
            self._startService.shutdown()
        except Exception as e:
            rospy.logerr("Fucked up in destructor of BehaviourBase: %s", e)
    
    def computeActivation(self):
        """
        This method should return the activation by the situation (from preconditions) as float [0 to 1]
        """
        try:
            return 1.0 if len(self._preconditions) == 0 else reduce(lambda x, y: x + y, (x.satisfaction for x in self._preconditions)) / len(self._preconditions)
        except AssertionError: # this probably comes from an uninitialized sensor or not matching activator for the sensor's data type in at least one precondition
            self._active = False
            return 0.0
    
    def computeSatisfaction(self):
        """
        This method should return the satisfaction of the preconditions (the readiness) as float [0 to 1].
        """
        try:
            return reduce(operator.mul, (x.satisfaction for x in self._preconditions), 1)
        except AssertionError: # this probably comes from an uninitialized sensor or not matching activator for the sensor's data type in at least one precondition
            self._active = False
            return 0.0
            
    def computeWishes(self):
        """
        This method should return a list of Wish messages indicating the desired sensor changes that would satisfy its preconditions.
        A Wish message is constructed from a string (sensor name) and a desire indicator (float, [-1 to 1]).
        """
        try:
            return [Wish(item[0].name, item[1]) for item in list(itertools.chain.from_iterable([x.getWishes() for x in self._preconditions]))]
        except AssertionError: # this probably comes from an uninitialized sensor or not matching activator for the sensor's data type in at least one precondition
            self._active = False
            return []
    
    def getProgress(self):
        """
        This method should return the progress of the current activities if isExecuting == True.
        It there is no current activity the value is ignored and may be filled with a dummy.
        """
        return 1.0
    
    def getStatus(self, request):
        self._active = True
        return GetStatusResponse(**{
                                  "activation"   : self.computeActivation(),
                                  "correlations" : [Correlation(name, value) for (name, value) in self._correlations.iteritems()],
                                  "satisfaction" : self.computeSatisfaction(),
                                  "threshold"    : self._readyThreshold,
                                  "wishes"       : self.computeWishes(),
                                  "isExecuting"  : self._isExecuting,
                                  "progress"     : self.getProgress(),
                                  "active"       : self._active
                                })
    
    def addPrecondition(self, precondition):
        '''
        This method adds a precondition to the behaviour.
        It is not mandatory to use this method at all but it may make development easier because the default implementations of computeActivation(), computeSatisfaction(), and computeWishes work with the preconditions added here.
        If you don't want to use this mechanism then you HAVE TO implement those yourself!
        There is an AND relationship between all elements (all have to be fulfilled so that the behaviour is ready)
        To enable OR semantics use the Disjunction object.
        '''
        if issubclass(type(precondition), conditions.Conditonal):
            self._preconditions.append(precondition)
        else:
            warnings.warn("That's no conditional object!")
    
    def startCallback(self, dummy):
        '''
        This method should switch the behaviour on.
        This method must not block.
        '''
        self._isExecuting = True
        self.action()
        return EmptyResponse()
    
    @property
    def correlations(self):
        return self._correlations
    
    @correlations.setter
    def correlations(self, correlations):
        '''
        This is for initializing the correlations.
        correlations must be a dict of form {sensor name <string> : correlation <float> [-1 to 1]}.
        '''
        self._correlations = correlations
    
    @property
    def readyThreshold(self):
        return self._readyThreshold
    
    @readyThreshold.setter
    def readyThreshold(self, threshold):
        self._readyThreshold = threshold
    
    def action(self):
        """
        This method should be overridden with one that actually does something.
        """
        self._isExecuting = True
        