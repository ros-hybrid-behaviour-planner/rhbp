'''
Created on 22.03.2017

@author: hrabia, wypler
'''
from __future__ import division # force floating point division when using plain /
from abc import ABCMeta, abstractmethod

import rospy

#from .managers import Manager has to be commented because of circular dependency problem
from .behaviours import Behaviour
from .pddl import create_valid_pddl_name


class AbstractActivationAlgorithm(object):
    """
    Abstract interface for activation calculation algorithms
    The particular algorithm implementation should be stateless, which means that no temporary calculation results
    are stored as member variables.
    """
    __metaclass__ = ABCMeta

    def __init__(self, manager, extensive_logging=False):
        """
        :param manager: reference to the manager object
        :type manager: Manager
        """
        self._manager = manager
        self._extensive_logging=extensive_logging

    @abstractmethod
    def compute_behaviour_activation_step(self, behaviour):
        """
        This method sums up all components of activation to compute the additional activation in this step.
        :param behaviour: the behaviour for which the activation is determined
        :type behaviour: Behaviour
        :return: the current step activation float
        """
        pass

    @abstractmethod
    def commit_behaviour_activation(self, behaviour):
        """
        Calculate the actual overall activation from the current activation step
        This method applies the activation of this iteration to the overall activation.
        :return: activation float
        """
        pass

    @abstractmethod
    def update_config(self,**kwargs):
        """
        Method for updating the algorithm configuration with named parameters
        :param kwargs:
        """
        pass


class ActivationAlgorithmFactory(object):
    """
    Factory for the registration and creation of activation algorithms
    """
    algorithms = {}

    @classmethod
    def register_algorithms(cls,id_algo, algo):
        """
        Register an algorithm in the factory under the given ID
        Algorithm has to be a subclass of AbstractActivationAlgorithm
        :param id_algo: ID for the algorithm
        :type id_algo: str
        :param algo: the algorithm class
        :type algo: AbstractActivationAlgorithm
        :return:
        """
        if not issubclass(algo, AbstractActivationAlgorithm):
            assert("Algo is not subclass of AbstractActivationAlgorithm")
        if cls.algorithms.has_key(id_algo):
            assert("Algorithm ID already in use")
        else:
            cls.algorithms[id_algo] = algo

    @classmethod
    def create_algorithm(cls, id_algo, manager):
        """
        Initialize the algorithm with the given ID
        :param id_algo: the id of the algo that should be created
        :type id_algo str
        :param manager: reference to the Manager
        :return: a specific instance of AbstractActivationAlgorithm
        """
        if not cls.algorithms.has_key(id_algo):
            raise LookupError("Cannot find algo_id: " + id_algo)
        else:
            return cls.algorithms[id_algo](manager)


class BaseActivationAlgorithm(AbstractActivationAlgorithm):
    """
    Base activation calculation as presented in paper TODO add reference of ICCAR paper
    """

    def __init__(self, manager, extensive_logging=False, create_log_files = False):
        super(BaseActivationAlgorithm, self).__init__(manager, extensive_logging=extensive_logging)

    def update_config(self, **kwargs):
        """
        Update the bias/weight configuration
        :param situation_bias:
        :param plan_bias:
        :param conflictor_bias:
        :param goal_bias:
        :param successor_bias:
        :param predecessor_bias:
        :param activation_decay: This reduces accumulated activation if the situation does not fit any more
        """
        self.situation_bias = kwargs['situation_bias']
        self.plan_bias=kwargs['plan_bias']
        self.conflictor_bias=kwargs['conflictor_bias']
        self.goal_bias=kwargs['goal_bias']
        self.succesor_bias=kwargs['successor_bias']
        self.predecessor_bias=kwargs['predecessor_bias']
        self._activation_decay=kwargs['activation_decay']

    def compute_behaviour_activation_step(self, behaviour):

        activation_precondition = self.get_activation_from_preconditions(behaviour)
        activation_goals = self.get_activation_from_goals(behaviour)[0]
        inhibition_goals = self.get_inhibition_from_goals(behaviour)[0]
        activation_predecessors = self.get_activation_from_predecessors(behaviour)[0]
        activation_successors = self.get_activation_from_successors(behaviour)[0]
        inhibition_conflictors = self.get_inhibition_from_conflictors(behaviour)[0]
        activation_plan = self.get_activation_from_plan(behaviour)[0]

        rospy.logdebug("\tactivation from preconditions: %s", activation_precondition)
        rospy.logdebug("\tactivation from goals: %s", activation_goals)
        rospy.logdebug("\tinhibition from goals: %s", inhibition_goals)
        rospy.logdebug("\tactivation from predecessors: %s", activation_predecessors)
        rospy.logdebug("\tactivation from successors: %s", activation_successors)
        rospy.logdebug("\tinhibition from conflicted: %s", inhibition_conflictors)
        rospy.logdebug("\tactivation from plan: %s", activation_plan)

        current_activation_step =  activation_precondition \
                                        + activation_goals \
                                        + inhibition_goals \
                                        + activation_predecessors \
                                        + activation_successors \
                                        + inhibition_conflictors \
                                        + activation_plan

        behaviour.current_activation_step = current_activation_step

        return current_activation_step


    def commit_behaviour_activation(self, behaviour):

        activation = behaviour.activation * self._activation_decay + behaviour.current_activation_step
        if activation < 0.0:
            activation = 0.0
        behaviour.activation = activation
        behaviour.current_activation_step= 0.0

        return activation

    def get_activation_from_preconditions(self, behaviour):
        """
        This methods computes the activation from the behaviour preconditions
        :param behaviour: the behaviour for which the activation is determined
        :type behaviour: Behaviour
        """
        return behaviour.activationFromPreconditions

    def get_activation_from_goals(self, behaviour):
        """
        This method computes the activation from goals.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        :param behaviour: the behaviour for which the activation is determined
        :type behaviour: Behaviour
        """
        activatedByGoals = []
        for goal in self._manager.activeGoals:
            # check for each sensor in the goal wishes for behaviours that have sensor effect correlations
            for (effect_name, indicator) in goal.wishes:
                # Make a list of all behaviours that are positively correlated to a wish of a goal (those behaviours will get activation from the goal).
                behavioursActivatedBySameGoal = [b for b in self._manager.activeBehaviours if any(
                    map(lambda x: x * indicator > 0.0, b.matchingCorrelations(effect_name)))]
                for correlation in behaviour.matchingCorrelations(effect_name):
                    if correlation * indicator > 0.0:  # This means we affect the sensor in a way that is desirable by the goal
                        totalActivation = correlation * indicator * self.goal_bias
                        if self._extensive_logging:
                            rospy.logdebug(
                                "Calculating activation from goals for %s. There is/are %d active behaviour(s) that support(s) %s via %s: %s with a total activation of %f. GoalBias is %f",
                                behaviour.name, len(behavioursActivatedBySameGoal), goal.name, effect_name,
                                behavioursActivatedBySameGoal, totalActivation, self.goal_bias)
                        activatedByGoals.append((goal, totalActivation / len(
                            behavioursActivatedBySameGoal)))  # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
        return (0.0,) if len(activatedByGoals) == 0 else (
        reduce(lambda x, y: x + y, (x[1] for x in activatedByGoals)), activatedByGoals)

    def get_inhibition_from_goals(self, behaviour):
        """
        This method computes the inhibition (actually, activation but negative sign!) caused by goals it conflicts with.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        :param behaviour: the behaviour for which the activation is determined
        :type behaviour: Behaviour
        """
        inhibitedByGoals = []
        for goal in self._manager.activeGoals:
            for (effect_name, indicator) in goal.wishes:
                # Make a list of all active behaviours that inhibit this goal.
                # Such behaviours are either negatively correlated to the goals wish (would prevent us from reaching the goal)
                # or the goal's condition has already been reached and the behaviour would undo it (goal's wish indicator is 0 but there is non-zero correlation of the behaviour to that particular sensor)
                behavioursInhibitedBySameGoal = [b for b in self._manager.activeBehaviours if any(
                    map(lambda x: x * indicator < 0.0 or (x * indicator == 0.0 and x != 0.0),
                        b.matchingCorrelations(effect_name)))]
                for correlation in behaviour.matchingCorrelations(effect_name):
                    if correlation * indicator < 0.0:  # This means we affect the sensor in a way that is not desirable by the goal
                        # We want the inhibition to be stronger if the condition that we would worsen is almost true.
                        # So we take -(1 - abs(indicator * correlation)) as the amount of total inhibition created by this conflict and divide it by the number of conflictors
                        totalInhibition = -(1 - abs(indicator)) * abs(correlation) * self.conflictor_bias
                        if self._extensive_logging:
                            rospy.logdebug(
                                "Calculating inhibition from goals for %s. There is/are %d behaviours(s) that worsen %s via %s: %s and a total inhibition score of %f",
                                behaviour.name, len(behavioursInhibitedBySameGoal), goal.name, effect_name,
                                behavioursInhibitedBySameGoal, totalInhibition)
                        inhibitedByGoals.append((goal, totalInhibition / len(
                            behavioursInhibitedBySameGoal)))  # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Note that this is negative, hence the name inhibition! Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
                    elif correlation != 0 and indicator == 0:  # That means the goals was achieved (wish indicator is 0) but we would undo that (because we are correlated to it somehow)
                        totalInhibition = -abs(correlation) * self.conflictor_bias
                        if self._extensive_logging:
                            rospy.logdebug(
                                "Calculating inhibition from goals for %s. There is/are %d behaviour(s) that undo %s via %s: %s and a total inhibition score of %f",
                                behaviour.name, len(behavioursInhibitedBySameGoal), goal.name, effect_name,
                                behavioursInhibitedBySameGoal, totalInhibition)
                        inhibitedByGoals.append((goal, totalInhibition / len(
                            behavioursInhibitedBySameGoal)))  # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Note that this is negative, hence the name inhibition! Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
        return (0.0,) if len(inhibitedByGoals) == 0 else (
        reduce(lambda x, y: x + y, (x[1] for x in inhibitedByGoals)), inhibitedByGoals)

    def get_activation_from_predecessors(self, behaviour):
        """
        This method computes the activation based on the fact that other behaviours can fulfill a precondition (wish) of this behaviour.
        This is scaled by the "readyness" (precondition satisfaction) of the predecessor as it makes only sense to activate
        a successor if it is likely that it is executable soon.
        :param behaviour: the behaviour for which the activation is determined
        :type behaviour: Behaviour
        """
        activatedByPredecessors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == self or not behaviour.executable:  # ignore ourselves and non-executable predecessors
                continue
            for (effect_name, indicator) in behaviour.wishes:  # this is what we wish from a predecessor
                # Make a list of all behaviours that share my wish (those will also get activated by the same predecessor). TODO could be improved by just counting --> less memory
                behavioursThatShareThisWish = [b for b in self._manager.activeBehaviours if
                                               any(map(lambda x: x * indicator > 0.0, b.matchingWishes(effect_name)))]
                for correlation in behaviour.matchingCorrelations(
                        effect_name):  # correlations that the behaviour (potential predecessor) has to the sensor of this wish
                    if correlation * behaviour.preconditionSatisfaction * indicator > 0.0:  # If a predecessor can satisfy my precondition
                        totalActivation = correlation * indicator * (
                        behaviour.activation / self._manager.totalActivation) * self.predecessor_bias
                        if self._extensive_logging:
                            rospy.logdebug(
                                "Calculating activation from predecessors for %s. There is/are %d active successor(s) of %s via %s: %s with total activation of %f",
                                behaviour.name, len(behavioursThatShareThisWish), behaviour.name, effect_name,
                                behavioursThatShareThisWish, totalActivation)
                        activatedByPredecessors.append((behaviour, effect_name, totalActivation / len(
                            behavioursThatShareThisWish)))  # The activation we get is the likeliness that our predecessor fulfills the preconditions soon. behavioursThatShareThisWish knows how many more behaviours will get activation from this predecessor so we distribute it equally
        return (0.0,) if len(activatedByPredecessors) == 0 else (
        reduce(lambda x, y: x + y, (x[2] for x in activatedByPredecessors)), activatedByPredecessors)

    def get_activation_from_successors(self, behaviour):
        """
        This method computes the activation this behaviour receives because it fulfills a precondition (is a predecessor)
         of a successor which has unfulfilled wishes.
        :param behaviour: the behaviour for which the activation is determined
        :type behaviour: Behaviour
        """
        activatedBySuccessors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == self or behaviour.executable:  # ignore ourselves and successors that are already executable
                continue
            for (effect_name, indicator) in behaviour.correlations:  # this is what can give to a successor
                # Make a list of all behaviours that are correlated to the same same sensor in the same way as we are. Those are also predecessors like us an get credit from the same successor.
                behavioursThatShareOurCorrelation = [b for b in self._manager.activeBehaviours if any(
                    map(lambda x: x * indicator > 0.0, b.matchingCorrelations(effect_name)))]
                for wish in behaviour.matchingWishes(effect_name):  # if we affect other behaviour's wishes somehow
                    if wish * indicator > 0:  # if we are a predecessor so we get activation from that successor
                        totalActivation = wish * indicator * (
                        behaviour.activation / self._manager.totalActivation) * self.successor_bias
                        if self._extensive_logging:
                            rospy.logdebug(
                                "Calculating activation from successors for %s. There is/are %d active predecessor(s) of %s via %s: %s and a total activation score of %f",
                                behaviour.name, len(behavioursThatShareOurCorrelation), behaviour.name, effect_name,
                                behavioursThatShareOurCorrelation, totalActivation)
                        activatedBySuccessors.append((behaviour, effect_name, totalActivation / len(
                            behavioursThatShareOurCorrelation)))  # The activation we get is our expected contribution to the fulfillment of our successors precondition. Actually only the value is needed but it is a tuple for debug purposes. len(behavioursThatShareOurCorrelation) is used to distribute activation among all predecessors
        return (0.0,) if len(activatedBySuccessors) == 0 else (
        reduce(lambda x, y: x + y, (x[2] for x in activatedBySuccessors)), activatedBySuccessors)

    def get_inhibition_from_conflictors(self, behaviour):
        """
        This method computes the inhibition (actually, activation but negative sign!) caused by other behaviours whose
        preconditions get antagonized when this behaviour runs.
        :param behaviour: the behaviour for which the activation is determined
        :type behaviour: Behaviour
        """
        inhibitionFromConflictors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == self:  # ignore ourselves
                continue
            for (effect_name, correlation) in behaviour.correlations:  # this is what we do to sensors
                for wish in behaviour.matchingWishes(effect_name):
                    # Make a list of all behaviours that have the same bad influence on other behaviours as we have.
                    # Such behaviours are either also negatively correlated another behaviour's wish as we are
                    # or would undo an already satisfied precondition of other behaviours as we would.
                    behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation = [b for b in
                                                                                          self._manager.activeBehaviours
                                                                                          if any(
                            map(lambda x: x * wish < 0.0 or (x * wish == 0.0 and x != 0.0),
                                b.matchingCorrelations(effect_name)))]
                    if wish * correlation < 0.0:  # if we make an existing conflict stronger
                        # We want the inhibition to be stronger if the condition that we would worsen is almost true.
                        # So we take -(1 - abs(wish * correlation)) as the amount of total inhibition created by this conflict and divide it by the number of conflictors
                        totalInhibition = -(1 - abs(wish)) * abs(correlation) * (
                        behaviour.activation / self._manager.totalActivation) * self.conflictor_bias
                        if self._extensive_logging:
                            rospy.logdebug(
                                "Calculating inhibition from conflicted for %s. %s is worsened via %s by %d behaviour(s): %s with a total score of %f",
                                behaviour.name, behaviour.name, effect_name,
                                len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation),
                                behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation, totalInhibition)
                        inhibitionFromConflictors.append((behaviour, effect_name, totalInhibition / len(
                            behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation)))  # Actually only the value is needed but it is a tuple for debug purposes. The length of behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation says how many more behaviours cause the same conflict to this conflictor so its inhibition shall be distributed among them.
                    elif wish * correlation == 0.0 and wish == 0:  # if we would change the currently existing good state for that behaviour (wish is zero but we have non-zero correlation to it)
                        totalInhibition = -abs(correlation) * (
                        behaviour.activation / self._manager.totalActivation) * self.conflictor_bias
                        if self._extensive_logging:
                            rospy.logdebug(
                                "Calculating inhibition from conflicted for %s. %s is undone via %s (wish: %f) by %d behaviour(s): %s by a total score of %f",
                                behaviour.name, behaviour.name, effect_name, wish,
                                len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation),
                                behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation, totalInhibition)
                        inhibitionFromConflictors.append((behaviour, effect_name, totalInhibition / len(
                            behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation)))  # The inhibition experienced is my bad influence (my correlation to this effect_name) times the wish of the other behaviour concerning this effect_nameme. Actually only the value is needed but it is a tuple for debug purposes. len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation) knows how many more behaviours cause the same harm to this conflictor so its inhibition shall be distributed among them.
        return (0.0,) if len(inhibitionFromConflictors) == 0 else (
        reduce(lambda x, y: x + y, (x[2] for x in inhibitionFromConflictors)), inhibitionFromConflictors)

    def get_activation_from_plan(self, behaviour):
        """
        This method computes the activation this behaviour receives because of its place on the plan.
        Behaviours at the top of the list will be activated most, other not so much.
        :param behaviour: the behaviour for which the activation is determined
        :type behaviour: Behaviour
        """
        if not self._manager.plan or ("cost" in self._manager.plan and self._manager.plan["cost"] == -1.0):
            return (0.0, 0xFF)
        own_pddl_name = create_valid_pddl_name(behaviour.name)
        for index in filter(lambda x: x >= self._manager.planExecutionIndex, sorted(
                self._manager.plan["actions"].keys())):  # walk along the plan starting at where we are
            if self._manager.plan["actions"][index] == own_pddl_name:  # if we are on the plan
                return (1 / (index - self._manager.planExecutionIndex + 1) * self.plan_bias, index)  # index in zero-based
        return (0.0, -1)


ActivationAlgorithmFactory.register_algorithms("default",BaseActivationAlgorithm)