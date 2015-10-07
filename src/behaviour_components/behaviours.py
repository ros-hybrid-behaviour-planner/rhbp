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
from behaviour_planner.msg import Wish, Correlation, Status
from behaviour_planner.srv import AddBehaviour, GetStatus, GetStatusResponse, Activate, ActivateResponse, Priority, PriorityResponse, GetPDDL, GetPDDLResponse
from util import PDDL

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
        self._isExecuting = False   # We get it via getStatus service of actual behaviour node
        self._interruptable = False # We get it via getStatus service of actual behaviour node
        self._progress = 0.0        # We get it via getStatus service of actual behaviour node
        self._readyThreshold = 0.0  # This is the threshold that the preconditionSatisfaction must reach in order for this behaviour to be executable. We get this value via getStatus service of actual behaviour node.
        self._activationDecay = 0.0 # This reduces accumulated activation if the situation does not fit any more
        self._active = True         # This indicates (if True) that there have been no severe issues in the actual behaviour node and the behaviour can be expected to be operational. If the actual behaviour reports active == False we will ignore it in activation computation.
        self._priority = 0          # The priority indicators are unsigned ints. The higher the more important
        self._manualStart = False   # If True the behaviour is started and cannot be switched off by the planner
        self._activated = True      # This member only exists as proxy for the corrsponding actual behaviour's property. It is here because of the comprehensive status message published each step by the manager for rqt
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
            status = getStatusRequest().status
            self._activationFromPreconditions = status.activation
            self._correlations = dict([(correlation.sensorName, correlation.indicator) for correlation in status.correlations])
            self._preconditionSatisfaction = status.satisfaction
            self._readyThreshold = status.threshold
            self._wishes = dict([(wish.sensorName, wish.indicator) for wish in status.wishes])
            if self._isExecuting == True and status.isExecuting == False:
                rospy.loginfo("%s finished. resetting activation", self._name)
                self._activation = 0.0
            self._isExecuting = status.isExecuting
            self._progress = status.progress
            self._active = status.active
            self._priority = status.priority
            self._interruptable = status.interruptable
            self._activated = status.activated
            if self._name != status.name:
                rospy.logerr("%s fetched a status message from a different behaviour: %s. This cannot happen!", self._name, status.name)
            rospy.logdebug("%s reports the following status:\nactivation %s\ncorrelations %s\nprecondition satisfaction %s\n ready threshold %s\nwishes %s\nactive %s\npriority %d\ninterruptable %s", self._name, self._activationFromPreconditions, self._correlations, self._preconditionSatisfaction, self._readyThreshold, self._wishes, self._active, self._priority, self._interruptable)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in GetStatus of %s: %s", self._name, e)  
    
    def fetchPDDL(self):
        '''
        This method fetches the status from the actual behaviour node via GetPDDLservice call
        '''
        rospy.logdebug("Waiting for service %s", self._name + 'PDDL')
        rospy.wait_for_service(self._name + 'PDDL')
        try:
            getPDDLRequest = rospy.ServiceProxy(self._name + 'PDDL', GetPDDL)
            pddl = getPDDLRequest()
            rospy.logdebug("\n%s", pddl)
            return PDDL(statement = pddl.statement, predicates = pddl.predicates, functions = pddl.functions)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in getPDDL of %s: %s", self._name, e)
            
    def getActivationFromGoals(self):
        '''
        This method computes the activation from goals.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        '''
        activatedByGoals = []
        for goal in self._manager.activeGoals:
            for (sensorName, indicator) in goal.wishes.iteritems():
                if sensorName in self._correlations.keys() and self._correlations[sensorName] * indicator > 0.0: # This means we affect the sensor in a way that is desirable by the goal
                    behavioursActivatedBySameGoal = [b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] * indicator > 0.0] # the variable name says it all
                    totalActivation = self._correlations[sensorName] * indicator * rospy.get_param("goalBias", 1.0)
                    rospy.logdebug("Calculating activation from goals for %s. There is/are %d active goal(s) that support(s) %s via %s: %s with a total activation of %f",  self.name, len(behavioursActivatedBySameGoal), goal.name, sensorName, behavioursActivatedBySameGoal, totalActivation)
                    activatedByGoals.append((goal, totalActivation / len(behavioursActivatedBySameGoal)))        # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
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
                    behavioursInhibitedBySameGoal = [b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] * indicator < 0] # the variable name says it all
                    # We want the inhibition to be stronger if the condition that we would worsen is almost true.
                    # So we take -(1 - abs(indicator) * self._correlations[sensorName]) as the amount of total inhibition created by this conflict and divide it by the number of conflictors
                    totalInhibition = -(1 - abs(indicator) * self._correlations[sensorName]) * rospy.get_param("goalBias", 1.0)
                    rospy.logdebug("Calculating inhibition from goals for %s. There is/are %d goal(s) that worsen %s via %s: %s and a total inhibition score of %f",  self.name, len(behavioursInhibitedBySameGoal), goal.name, sensorName, behavioursInhibitedBySameGoal, totalInhibition)
                    inhibitedByGoals.append((goal, totalInhibition / len(behavioursInhibitedBySameGoal)))      # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Note that this is negative, hence the name inhibition! Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
                elif sensorName in self._correlations.keys() and self._correlations[sensorName] != 0 and self._correlations[sensorName] * indicator == 0:  # That means the goals was achieved but we would undo that
                    behavioursInhibitedBySameGoal = [b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and self._correlations[sensorName] != 0 and b.correlations[sensorName] * indicator == 0] # the variable name says it all
                    totalInhibition = -abs(self._correlations[sensorName]) * rospy.get_param("goalBias", 1.0)
                    rospy.logdebug("Calculating inhibition from goals for %s. There is/are %d behaviour(s) that undo %s via %s: %s and a total inhibition score of %f",  self.name, len(behavioursInhibitedBySameGoal), goal.name, sensorName, behavioursInhibitedBySameGoal, totalInhibition)
                    inhibitedByGoals.append((goal, totalInhibition / len(behavioursInhibitedBySameGoal)))      # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Note that this is negative, hence the name inhibition! Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
        return (0.0,) if len(inhibitedByGoals) == 0 else (reduce(lambda x, y: x + y, (x[1] for x in inhibitedByGoals)) / len(inhibitedByGoals), inhibitedByGoals)
    
    def getActivationFromPredecessors(self):
        '''
        This method computes the activation based on the fact that other behaviours can fulfill a precondition of this behaviour.
        This is scaled by the "readyness" (precondition satisfaction) of the predecessor as it makes only sense to activate a successor if it is likely that it is executable soon.
        '''
        activatedByPredecessors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == self or not behaviour.executable: # ignore ourselves and non-executable predecessors
                continue
            for (sensorName, indicator) in self._wishes.iteritems():
                if sensorName in behaviour.correlations.keys() and behaviour.correlations[sensorName] *  behaviour.preconditionSatisfaction * indicator > 0.0:  # If a predecessor can satisfy my precondition
                    successorsAffectedByThisPredecessorsCorrellation = [b for b in self._manager.activeBehaviours if sensorName in b.wishes.keys() and b.wishes[sensorName] * indicator > 0.0] # the variable name says it all
                    totalActivation = behaviour.correlations[sensorName] * indicator * behaviour.activation * rospy.get_param("behaviourBias", 0.25)
                    rospy.logdebug("Calculating activation from predecessors for %s. There is/are %d active successor(s) of %s via %s: %s with total activation of %f",  self._name, len(successorsAffectedByThisPredecessorsCorrellation), behaviour.name, sensorName, successorsAffectedByThisPredecessorsCorrellation, totalActivation)
                    activatedByPredecessors.append((behaviour, sensorName, totalActivation / len(successorsAffectedByThisPredecessorsCorrellation))) # The activation we get is the likeliness that our predecessor fulfills the preconditions soon. numSuccessorsOfThatPredecessorAffectedByThisCorrelation knows how many more behaviours will get activation from this predecessor so we distribute it equally
        return (0.0,) if len(activatedByPredecessors) == 0 else (reduce(lambda x, y: x + y, (x[2] for x in activatedByPredecessors)), activatedByPredecessors)

    def getActivationFromSuccessors(self):
        '''
        This method computes the activation this behaviour receives because it fulfills a precondition (is a predecessor) of a successor which has unfulfilled wishes.
        '''
        activatedBySuccessors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == self or behaviour.executable: # ignore ourselves and successors that are already executable
                continue
            for (sensorName, indicator) in self._correlations.iteritems(): # this is what we do to sensors
                if sensorName in behaviour.wishes.keys():                  # if we affect other behaviours wishes somehow
                    if behaviour.wishes[sensorName] * indicator > 0:       # if we are a predecessor so we get activation from that successor
                        # TODO: The following list is incorrect. It should list all behaviours that have sensorName in their wishes with the same indicator as out correlation has to this sensor
                        predecessorsAffectingSuccessorsByThisCorrelation = [b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] * indicator > 0.0] # the variable name says it all
                        totalActivation = behaviour.wishes[sensorName] * indicator * behaviour.activation * rospy.get_param("behaviourBias", 0.25)
                        rospy.logdebug("Calculating activation from successors for %s. There is/are %d active predecessor(s) of %s via %s: %s and a total activation score of %f", self._name, len(predecessorsAffectingSuccessorsByThisCorrelation), behaviour.name, sensorName, predecessorsAffectingSuccessorsByThisCorrelation, totalActivation)
                        activatedBySuccessors.append((behaviour, sensorName, totalActivation / len(predecessorsAffectingSuccessorsByThisCorrelation))) # The activation we get is our expected contribution to the fulfillment of our successors precondition. Actually only the value is needed but it is a tuple for debug purposes. len(predecessorsAffectingSuccessorsByThisCorrelation) is used to distribute activation among all predecessors
        return (0.0,) if len(activatedBySuccessors) == 0 else (reduce(lambda x, y: x + y, (x[2] for x in activatedBySuccessors)) / len(activatedBySuccessors), activatedBySuccessors)

    def getInhibitionFromConflicted(self):
        '''
        This method computes the inhibition (actually, activation but negative sign!) caused by other behaviours whose preconditions get antagonized when this behaviour runs.
        '''
        inhibitionFromConflictors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == self: # ignore ourselves
                continue
            for (sensorName, indicator) in self._correlations.iteritems(): # this is what we do to sensors
                if sensorName in behaviour.wishes.keys():                  # if we affect other behaviours somehow
                    if behaviour.wishes[sensorName] * indicator < 0:       # if we make an existing conflict stronger
                        behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation = [b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] * behaviour.wishes[sensorName] < 0] # the variable name says it all
                        # We want the inhibition to be stronger if the condition that we would worsen is almost true.
                        # So we take -(1 - abs(behaviour.wishes[sensorName]) * abs(indicator)) as the amount of total inhibition created by this conflict and divide it by the number of conflictors
                        totalInhibition = -(1 - abs(behaviour.wishes[sensorName]) * abs(indicator)) * rospy.get_param("behaviourBias", 0.25)
                        rospy.logdebug("Calculating inhibition from conflicted for %s. %s is worsened via %s by %d behaviour(s): %s with a total score of %f", self.name, behaviour.name, sensorName, len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation), behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation, totalInhibition)
                        inhibitionFromConflictors.append((behaviour, sensorName, totalInhibition / len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation))) #  Actually only the value is needed but it is a tuple for debug purposes. The length of behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation says how many more behaviours cause the same conflict to this conflictor so its inhibition shall be distributed among them.
                    elif behaviour.wishes[sensorName] * indicator == 0 and indicator != 0:    # if we would change the currently existing good state for that behaviour
                        behavioursThatWouldChangeTheCurrentyGoodCondition = [b for b in self._manager.activeBehaviours if sensorName in b.correlations.keys() and b.correlations[sensorName] != 0 and b.correlations[sensorName] * behaviour.wishes[sensorName] == 0] # the variable name says it all
                        totalInhibition = -abs(indicator) * behaviour.activation * rospy.get_param("behaviourBias", 0.25)
                        rospy.logdebug("Calculating inhibition from conflicted for %s. %s is undone via %s by %d behaviour(s): %s by a total score of %f", self.name, behaviour.name, sensorName, len(behavioursThatWouldChangeTheCurrentyGoodCondition), behavioursThatWouldChangeTheCurrentyGoodCondition, totalInhibition)
                        inhibitionFromConflictors.append((behaviour, sensorName, totalInhibition / len(behavioursThatWouldChangeTheCurrentyGoodCondition))) # The inhibition experienced is my bad influence (my correlation to this sensorName) times the wish of the other behaviour concerning this sensorName. Actually only the value is needed but it is a tuple for debug purposes. len(behavioursThatWouldChangeTheCurrentyGoodCondition) knows how many more behaviours cause the same conflict to this conflictor so its inhibition shall be distributed among them.
        return (0.0,) if len(inhibitionFromConflictors) == 0 else (reduce(lambda x, y: x + y, (x[2] for x in inhibitionFromConflictors)) / len(inhibitionFromConflictors), inhibitionFromConflictors)

    def computeActivation(self):
        '''
        This method sums up all components of activation to compute the additional activation in this step.
        '''
        if self._active:
            self.__currentActivationStep = self._activationFromPreconditions \
            + self.getActivationFromGoals()[0] \
            + self.getInhibitionFromGoals()[0] \
            + self.getActivationFromPredecessors()[0] \
            + self.getActivationFromSuccessors()[0] \
            + self.getInhibitionFromConflicted()[0]
        else:
            return 0.0
    
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
        This method calls the start service of the actual behaviour.
        It is expected that this service does not block.
        '''
        assert not self._isExecuting
        self._isExecuting = True
        try:
            rospy.logdebug("Waiting for service %s", self._name + 'Start')
            rospy.wait_for_service(self._name + 'Start')
            startRequest = rospy.ServiceProxy(self._name + 'Start', Empty)
            startRequest()
            rospy.loginfo("Started action of %s", self._name)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception while calling %s: %s", self._name + 'Start', e)
    
    def stop(self):
        '''
        This method calls the stop service of the actual behaviour.
        It is expected that this service does not block.
        '''
        assert self._isExecuting
        try:
            rospy.logdebug("Waiting for service %s", self._name + 'Stop')
            rospy.wait_for_service(self._name + 'Stop')
            stopRequest = rospy.ServiceProxy(self._name + 'Stop', Empty)
            stopRequest()
            rospy.loginfo("Stopping action of %s", self._name)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception while calling %s: %s", self._name + 'Stop', e)
        self._isExecuting = True # I should possibly set this at the end of try block but if that fails we are screwed anyway
            
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
    def manualStart(self):
        return self._manualStart
    
    @manualStart.setter
    def manualStart(self, status):
        self._manualStart = status
        
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
    def activated(self):
        return self._activated
    
    @property
    def priority(self):
        return self._priority
    
    @property
    def progress(self):
        return self._progress
    
    @property
    def interruptable(self):
        return self._interruptable
    
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
        self._getStatusService = rospy.Service(self._name + 'GetStatus', GetStatus, self.getStatus)
        self._startService = rospy.Service(self._name + 'Start', Empty, self.startCallback)
        self._stopService = rospy.Service(self._name + 'Stop', Empty, self.stopCallback)
        self._activateService = rospy.Service(self._name + 'Activate', Activate, self.activateCallback)
        self._pddlService = rospy.Service(self._name + 'PDDL', GetPDDL, self.pddlCallback)
        self._priorityService = rospy.Service(self._name + 'Priority', Priority, self.setPriorityCallback)
        self._preconditions = kwargs["preconditions"] if "preconditions" in kwargs else [] # This are the preconditions for the behaviour. They may not be used but the default implementations of computeActivation(), computeSatisfaction(), and computeWishes work them. See addPrecondition()
        self._isExecuting = False  # Set this to True if this behaviour is selected for execution.
        self._correlations = kwargs["correlations"] if "correlations" in kwargs else [] # Stores sensor correlations in list form. Expects a list of utils.Effect objects with following meaning: sensorName -> name of affected sensor, indicator -> value between -1 and 1 encoding how this sensor  is affected. 1 Means high positive correlation to the value or makes it become True, -1 the opposite and 0 does not affect anything. Optional condition -> a piece of pddl when this effect happens. # Be careful with the sensorName! It has to actually match something that exists!
        self._readyThreshold = kwargs["readyThreshold"] if "readyThreshold" in kwargs else 0.8 # This is the threshold that the preconditions must reach in order for this behaviour to be executable.
        self._plannerPrefix = kwargs["plannerPrefix"] if "plannerPrefix" in kwargs else "" # if you have multiple planners in the same ROS environment use a prefix to name the right one.
        self._interruptable = kwargs["interruptable"] if "interruptable" in kwargs else False # The name says it all
        self._actionCost = kwargs["actionCost"] if "actionCost" in kwargs else 1.0 # This is the threshold that the preconditions must reach in order for this behaviour to be executable.
        self._priority = kwargs["priority"] if "priority" in kwargs else 0 # The priority indicators are unsigned ints. The higher the more important
        self._active = True # if anything in the behaviour is not initialized or working properly this must be set to False and communicated via getStatus service. The value of this variable is set to self._activated at the start of each status poll and should be set to False in case of errors.
        self._activated = True # The activate Service sets the value of this property.

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
            self._stopService.shutdown()
            self._activateService.shutdown()
            self._priorityService.shutdown()
            self._pddlService.shutdown()
        except Exception as e:
            rospy.logerr("Error in destructor of BehaviourBase: %s", e)
    
    def computeActivation(self):
        """
        This method should return the activation by the situation (from preconditions) as float [0 to 1]
        Note that there may be optional sensors 
        """
        activations = [] # this list gets filled with the activations of the individual preconditions
        for p in filter(lambda x: x.optional == False, self._preconditions): # check mandatory ones first because if there is an error we don't need to look at the optional ones at all
            try:
                activations.append(p.satisfaction)
            except AssertionError: # this probably comes from an uninitialized sensor or not matching activator for the sensor's data type in at least one precondition
                self._active = False
                return 0.0
        for p in filter(lambda x: x.optional == True, self._preconditions): # now check optional sensors
            try:
                activations.append(p.satisfaction)
            except AssertionError: # we don't care about errors in optional sensors
                pass
        return 1.0 if len(activations) == 0 else reduce(lambda x, y: x + y, activations) / len(activations)
            
    
    def computeSatisfaction(self):
        """
        This method should return the satisfaction of the preconditions (the readiness) as float [0 to 1].
        If there are functioning optional sensors they are also handled equally like mandatory ones (so they could screw up the overall satisfaction) but if they fail they are just ignored.
        """
        satisfactions = [] # this list gets filled with the satisfactions of the individual preconditions
        for p in filter(lambda x: x.optional == False, self._preconditions): # check mandatory ones first because if there is an error we don't need to look at the optional ones at all
            try:
                satisfactions.append(p.satisfaction)
            except AssertionError: # this probably comes from an uninitialized sensor or not matching activator for the sensor's data type in at least one precondition
                self._active = False
                return 0.0
        for p in filter(lambda x: x.optional == True, self._preconditions): # now check optional sensors
            try:
                satisfactions.append(p.satisfaction)
            except AssertionError: # we don't care about errors in optional sensors
                pass
        return reduce(operator.mul, satisfactions, 1)
            
    def computeWishes(self):
        """
        This method should return a list of Wish messages indicating the desired sensor changes that would satisfy its preconditions.
        A Wish message is constructed from a string (sensor name) and a desire indicator (float, [-1 to 1]).
        """
        wishes = [] # this list gets filled with the satisfactions of the individual preconditions
        for p in filter(lambda x: x.optional == False, self._preconditions): # check mandatory ones first because if there is an error we don't need to look at the optional ones at all
            try:
                wishes = list(itertools.chain(wishes, p.getWishes()))
            except AssertionError: # this probably comes from an uninitialized sensor or not matching activator for the sensor's data type in at least one precondition
                self._active = False
                return []
        for p in filter(lambda x: x.optional == True, self._preconditions): # now check optional sensors
            try:
                wishes = list(itertools.chain(wishes, p.getWishes()))
            except AssertionError: # we don't care about errors in optional sensors
                pass
        return [Wish(item[0].name, item[1]) for item in wishes]
    
    def getProgress(self):
        """
        This method should return the progress of the current activities if isExecuting == True.
        It there is no current activity the value is ignored and may be filled with a dummy.
        """
        return 0.5
    
    def getActionPDDL(self):
        """
        This method should produce a valid PDDL action snippet suitable for FastDownward (http://www.fast-downward.org/PddlSupport)
        """
        pddl = PDDL(statement =  "(:action {0}\n:parameters ()\n".format(self._name), functions = "costs")
        preconds = [x.getPDDL() for x in self._preconditions]
        pddl.predicates = set(itertools.chain.from_iterable(map(lambda x: x.predicates, preconds))) # unites all predicates in preconditions
        pddl.functions = pddl.functions.union(*map(lambda x: x.functions, preconds)) # unites all functions in preconditions
        if len(preconds) > 1:
            pddl.statement += ":precondition (and " + " ".join(map(lambda x: x.statement, preconds)) + ")\n"
        elif len(preconds) == 1:
            pddl.statement += ":precondition " + preconds[0].statement + "\n"
        effects = [x.getPDDL() for x in self._correlations]
        pddl.predicates = pddl.predicates.union(*map(lambda x: x.predicates, effects)) # adds predicates from the effects
        pddl.functions = pddl.functions.union(*map(lambda x: x.functions, effects)) # adds functions from the effects
        if len(effects) > 1:
            pddl.statement += ":effect (and (increase (costs) {0}) ".format(self._actionCost) + " ".join(map(lambda x: x.statement, effects)) + ")\n"
        elif len(effects) == 1:
            pddl.statement += ":effect (and (increase (costs) {0}) {1})\n".format(self._actionCost, effects[0].statement)
        pddl.statement += ")\n"
        return pddl

    def pddlCallback(self, dummy):
        pddl = self.getActionPDDL()
        return GetPDDLResponse(**{"statement" : pddl.statement, "predicates" : list(pddl.predicates), "functions" : list(pddl.functions)})
    
    def getStatus(self, request):
        self._active = self._activated
        status = Status(**{
                           "name"         : self._name, # this is sent for sanity check and planner status messages only
                           "activation"   : self.computeActivation(),
                           "correlations" : [Correlation(x.sensorName, x.indicator) for x in self._correlations],
                           "satisfaction" : self.computeSatisfaction(),
                           "threshold"    : self._readyThreshold,
                           "wishes"       : self.computeWishes(),
                           "isExecuting"  : self._isExecuting,
                           "progress"     : self.getProgress(),
                           "active"       : self._active, # if any of the above methods failed this property has been set to False by now
                           "priority"     : self._priority,
                           "interruptable": self._interruptable,
                           "activated"    : self._activated
                          })
        return GetStatusResponse(status)
    
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
        self.start()
        return EmptyResponse()
    
    def stopCallback(self, dummy):
        '''
        This method should switch the behaviour off.
        This method must not block.
        '''
        self.stop()        
        self._isExecuting = False
        return EmptyResponse()
    
    def activateCallback(self, request):
        '''
        This method activates or deactivates the behaviour.
        This method must not block.
        '''
        self._activated = request.active
        if self._activated == False:
            self.stop() 
            self._isExecuting = False
        return ActivateResponse()
    
    def setPriorityCallback(self, request):
        self._priority = request.priority
        return PriorityResponse()
    
    @property
    def correlations(self):
        return self._correlations
    
    @correlations.setter
    def correlations(self, correlations):
        '''
        This is for initializing the correlations.
        correlations must be an util.Effect object.
        '''
        self._correlations = correlations
    
    @property
    def readyThreshold(self):
        return self._readyThreshold
    
    @readyThreshold.setter
    def readyThreshold(self, threshold):
        self._readyThreshold = threshold
    
    @property
    def priority(self):
        return self._priority
    
    @priority.setter
    def priority(self, priority):
        self._priority = priority
    
    @property
    def interruptable(self):
        return self._interruptable
    
    @interruptable.setter
    def interruptable(self, interruptable):
        self._interruptable = interruptable
    
    def start(self):
        """
        This method should be overridden with one that actually does something.
        """
        pass
    
    def stop(self):
        """
        This method should be overridden with one that actually does something.
        """
        pass       
