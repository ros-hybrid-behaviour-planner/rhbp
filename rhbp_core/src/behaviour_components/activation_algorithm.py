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
import numpy
import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.planning')
from rhbp_core.msg import InputState, ActivationState
from rhbp_core.srv import GetActivation
import roslaunch
from reinforcement_component.rl_component import RLComponent

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
    def compute_behaviour_activation_step(self, ref_behaviour):
        """
        This method sums up all components of activation to compute the additional activation in this step.
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        :return: the current step activation float
        """
        pass

    @abstractmethod
    def commit_behaviour_activation(self, ref_behaviour):
        """
        Calculate the actual overall activation from the current activation step
        This method applies the activation of this iteration to the overall activation.
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
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
    def register_algorithm(cls, id_algo, algo):
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
    Base activation calculation mostly as presented in
    Towards Goal-driven Behaviour Control of Multi-Robot Systems
    Christopher-Eyk Hrabia, Stephan Wypler, Sahin Albayrak
    In: 2017 IEEE 3nd International Conference on Control, Automation and Robotics (ICCAR); 2017

    Formulas for the inhibition calculations have been changed/improved
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
        self._situation_bias = kwargs['situation_bias']
        self._plan_bias=kwargs['plan_bias']
        self._conflictor_bias=kwargs['conflictor_bias']
        self._goal_bias=kwargs['goal_bias']
        self._successor_bias=kwargs['successor_bias']
        self._predecessor_bias=kwargs['predecessor_bias']
        self._activation_decay=kwargs['activation_decay']

    def compute_behaviour_activation_step(self, ref_behaviour):

        activation_precondition = self.get_activation_from_preconditions(ref_behaviour)
        activation_goals = self.get_activation_from_goals(ref_behaviour)[0]
        inhibition_goals = self.get_inhibition_from_goals(ref_behaviour)[0]
        activation_predecessors = self.get_activation_from_predecessors(ref_behaviour)[0]
        activation_successors = self.get_activation_from_successors(ref_behaviour)[0]
        inhibition_conflictors = self.get_inhibition_from_conflictors(ref_behaviour)[0]
        activation_plan = self.get_activation_from_plan(ref_behaviour)[0]
        # TODO add here the activation from the rl-component
        # TODO e.g. rl_componenet.model.feed_forward()
        # TODO needs reference to rl_component
        # TODO rl-componenet should save old activation . and a converter for behavior to index
        #self._manager.rl_component.get_model_parameters()
        #activation_rl = self._manager.rl_component.get_rl_activation(ref_behaviour)

        rhbplog.loginfo("\t%s: activation from preconditions: %s", ref_behaviour, activation_precondition)
        rhbplog.loginfo("\t%s: activation from goals: %s", ref_behaviour, activation_goals)
        rhbplog.loginfo("\t%s: inhibition from goals: %s", ref_behaviour, inhibition_goals)
        rhbplog.loginfo("\t%s: activation from predecessors: %s", ref_behaviour, activation_predecessors)
        rhbplog.loginfo("\t%s: activation from successors: %s", ref_behaviour, activation_successors)
        rhbplog.loginfo("\t%s: inhibition from conflicted: %s", ref_behaviour, inhibition_conflictors)
        rhbplog.loginfo("\t%s: activation from plan: %s", ref_behaviour, activation_plan)

        current_activation_step = activation_precondition \
                                  + activation_goals \
                                  + inhibition_goals \
                                  + activation_predecessors \
                                  + activation_successors \
                                  + inhibition_conflictors \
                                  + activation_plan

        ref_behaviour.current_activation_step = current_activation_step

        return current_activation_step

    def commit_behaviour_activation(self, ref_behaviour):

        activation = ref_behaviour.activation * self._activation_decay + ref_behaviour.current_activation_step
        if activation < 0.0:
            activation = 0.0
        ref_behaviour.activation = activation
        ref_behaviour.current_activation_step = 0.0

        return activation

    def get_activation_from_preconditions(self, ref_behaviour):
        """
        This methods computes the activation from the behaviour preconditions
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        return ref_behaviour.activationFromPreconditions

    def get_activation_from_goals(self, ref_behaviour):
        """
        This method computes the activation from goals.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        activatedByGoals = []
        for goal in self._manager.activeGoals:
            # check for each sensor in the goal wishes for behaviours that have sensor effect correlations
            for wish in goal.wishes:
                wish_name = wish.get_pddl_effect_name() #TODO this has to be reconsidered
                wish_indicator = wish.indicator
                # Make a list of all behaviours that are positively correlated to a wish of a goal (those behaviours will get activation from the goal).
                behavioursActivatedBySameGoal = [b for b in self._manager.activeBehaviours if any(
                    map(lambda x: x * wish_indicator > 0.0, self._matching_effect_indicators(ref_behaviour=b, effect_name=wish_name)))]
                amount_activated_behaviours = len(behavioursActivatedBySameGoal) if len(behavioursActivatedBySameGoal) > 0 else 1
                for correlation_indicator in self._matching_effect_indicators(ref_behaviour=ref_behaviour, effect_name=wish_name):
                    if correlation_indicator * wish_indicator > 0.0:  # This means we affect the sensor in a way that is desirable by the goal
                        totalActivation = correlation_indicator * wish_indicator * self._goal_bias
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating activation from goals for %s. There is/are %d active behaviour(s) that support(s) %s via %s: %s with a total activation of %f. GoalBias is %f",
                                ref_behaviour.name, amount_activated_behaviours, goal.name, wish_name,
                                behavioursActivatedBySameGoal, totalActivation, self._goal_bias)
                        # The activation we get from that is the product of the correlation we have to this Sensor and
                        # the Goal's desired change of this Sensor. Actually, we are only interested in the value itself
                        # but for debug purposed we make it a tuple including the goal itself
                        activatedByGoals.append((goal, totalActivation / amount_activated_behaviours))
        return (0.0,) if len(activatedByGoals) == 0 else (
        reduce(lambda x, y: x + y, (x[1] for x in activatedByGoals)), activatedByGoals)

    def get_inhibition_from_goals(self, ref_behaviour):
        """
        This method computes the inhibition (actually, activation but negative sign!) caused by goals it conflicts with.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        inhibitedByGoals = []
        for goal in self._manager.activeGoals:
            for wish in goal.wishes:
                wish_name = wish.get_pddl_effect_name()  # TODO this has to be reconsidered
                wish_indicator = wish.indicator
                # Make a list of all active behaviours that inhibit this goal.
                # Such behaviours are either negatively correlated to the goals wish (would prevent us from reaching the goal)
                # or the goal's condition has already been reached and the behaviour would undo it (goal's wish indicator is 0 but there is non-zero correlation of the behaviour to that particular sensor)
                behavioursInhibitedBySameGoal = [b for b in self._manager.activeBehaviours if any(
                    map(lambda x: x * wish_indicator < 0.0 or (x * wish_indicator == 0.0 and x != 0.0),
                        self._matching_effect_indicators(ref_behaviour=b, effect_name=wish_name)))]
                amount_conflictor_behaviours = len(behavioursInhibitedBySameGoal) if len(behavioursInhibitedBySameGoal) > 0 else 1
                for correlation_indicator in self._matching_effect_indicators(ref_behaviour=ref_behaviour, effect_name=wish_name):
                    if correlation_indicator * wish_indicator < 0.0:  # This means we affect the sensor in a way that is not desirable by the goal
                        # We want the inhibition to be stronger if the condition that we would worsen is almost true.
                        # So we take -(1 - abs(wish * correlation)) as the amount of total inhibition created by this
                        # (for all wishes*effect != 1), in the other case we take a fixed small inhibition of -0.1
                        # conflict and divide it by the number of conflictors
                        if (abs(wish_indicator * correlation_indicator)) == 1:  # without this condition it would result in an inhibition of 0
                            totalInhibition = -0.1  # just take a fixed small inhibition here
                        else:
                            totalInhibition = -(1 - abs(wish_indicator * correlation_indicator))
                        totalInhibition = totalInhibition * self._conflictor_bias # TODO somehow strange that we use the conflictor bias here
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating inhibition from goals for %s. There is/are %d behaviours(s) that worsen %s via %s: %s and a total inhibition score of %f",
                                ref_behaviour.name, amount_conflictor_behaviours, goal.name, wish_name,
                                behavioursInhibitedBySameGoal, totalInhibition)
                        # The activation we get from that is the product of the correlation we have to this Sensor and
                        # the Goal's desired change of this Sensor. Note that this is negative, hence the name inhibition!
                        # Actually, we are only interested in the value itself but for debug purposed we make it a tuple
                        # including the goal itself
                        inhibitedByGoals.append((goal, totalInhibition / amount_conflictor_behaviours))
                    elif correlation_indicator != 0 and wish_indicator == 0:  # That means the goals was achieved (wish indicator is 0) but we would undo that (because we are correlated to it somehow)
                        totalInhibition = -abs(correlation_indicator) * self._conflictor_bias
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating inhibition from goals for %s. There is/are %d behaviour(s) that undo %s via %s: %s and a total inhibition score of %f",
                                ref_behaviour.name, amount_conflictor_behaviours, goal.name, wish_name,
                                behavioursInhibitedBySameGoal, totalInhibition)
                        # The activation we get from that is the product of the correlation we have to this Sensor and
                        # the Goal's desired change of this Sensor. Note that this is negative, hence the name inhibition!
                        # Actually, we are only interested in the value itself but for debug purposed we make it a tuple
                        # including the goal itself
                        inhibitedByGoals.append((goal, totalInhibition / amount_conflictor_behaviours))
        return (0.0,) if len(inhibitedByGoals) == 0 else (
        reduce(lambda x, y: x + y, (x[1] for x in inhibitedByGoals)), inhibitedByGoals)

    def get_activation_from_predecessors(self, ref_behaviour):
        """
        This method computes the activation based on the fact that other behaviours can fulfill a precondition (wish) of this behaviour.
        This is scaled by the "readyness" (precondition satisfaction) of the predecessor as it makes only sense to activate
        a successor if it is likely that it is executable soon.
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        activatedByPredecessors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == ref_behaviour or not behaviour.executable:  # ignore ourselves and non-executable predecessors
                continue
            for wish in ref_behaviour.wishes:  # this is what we wish from a predecessor
                wish_name = wish.get_pddl_effect_name()  # TODO this has to be reconsidered
                wish_indicator = wish.indicator
                # Make a list of all behaviours that share my wish (those will also get activated by the same predecessor). TODO could be improved by just counting --> less memory
                behavioursThatShareThisWish = [b for b in self._manager.activeBehaviours if
                                               any(map(lambda x: x * wish_indicator > 0.0, self._matching_wishes_indicators(ref_behaviour=b, effect_name=wish_name)))]
                amount_wish_sharing_behaviours = len(behavioursThatShareThisWish) if len(behavioursThatShareThisWish) > 0 else 1
                # correlations that the behaviour (potential predecessor) has to the sensor of this wish
                for correlation_indicators in self._matching_effect_indicators(ref_behaviour=behaviour, effect_name=wish_name):
                    # If a predecessor can satisfy my precondition
                    if correlation_indicators * behaviour.preconditionSatisfaction * wish_indicator > 0.0:
                        totalActivation = correlation_indicators * wish_indicator * (
                        behaviour.activation / self._manager.totalActivation) * self._predecessor_bias
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating activation from predecessors for %s. There is/are %d active successor(s) of %s via %s: %s with total activation of %f",
                                ref_behaviour.name, amount_wish_sharing_behaviours, behaviour.name, wish_name,
                                behavioursThatShareThisWish, totalActivation)
                        # The activation we get is the likeliness that our predecessor fulfills the preconditions soon.
                        # behavioursThatShareThisWish knows how many more behaviours will get activation from this
                        # predecessor so we distribute it equally
                        activatedByPredecessors.append((behaviour, wish_name, totalActivation / amount_wish_sharing_behaviours))
        return (0.0,) if len(activatedByPredecessors) == 0 else (
        reduce(lambda x, y: x + y, (x[2] for x in activatedByPredecessors)), activatedByPredecessors)

    def get_activation_from_successors(self, ref_behaviour):
        """
        This method computes the activation this behaviour receives because it fulfills a precondition (is a predecessor)
         of a successor which has unfulfilled wishes.
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        activatedBySuccessors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == ref_behaviour or behaviour.executable:  # ignore ourselves and successors that are already executable
                continue
            for effect in ref_behaviour.correlations:  # this is what can give to a successor
                effect_name = effect.get_pddl_effect_name() # TODO this has to be reconsidered
                effect_indicator = effect.indicator
                # Make a list of all behaviours that are correlated to the same same sensor in the same way as we are. Those are also predecessors like us an get credit from the same successor.
                behavioursThatShareOurCorrelation = [b for b in self._manager.activeBehaviours if any(
                    map(lambda x: x * effect_indicator > 0.0, self._matching_effect_indicators(ref_behaviour=b, effect_name=effect_name)))]
                amount_behaviours_sharing_correlation = len(behavioursThatShareOurCorrelation) if len(behavioursThatShareOurCorrelation) > 0 else 1
                for wish_indicator in self._matching_wishes_indicators(ref_behaviour=behaviour, effect_name=effect_name):  # if we affect other behaviour's wishes somehow
                    if wish_indicator * effect_indicator > 0:  # if we are a predecessor so we get activation from that successor
                        totalActivation = wish_indicator * effect_indicator * (
                        behaviour.activation / self._manager.totalActivation) * self._successor_bias
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating activation from successors for %s. There is/are %d active predecessor(s) of %s via %s: %s and a total activation score of %f",
                                ref_behaviour.name, amount_behaviours_sharing_correlation, behaviour.name, effect_name,
                                behavioursThatShareOurCorrelation, totalActivation)
                        # The activation we get is our expected contribution to the fulfillment of our successors
                        # precondition. Actually only the value is needed but it is a tuple for debug purposes.
                        # amount_behaviours_sharing_correlation is used to distribute activation among all predecessors
                        activatedBySuccessors.append((behaviour, effect_name, totalActivation / amount_behaviours_sharing_correlation))
        return (0.0,) if len(activatedBySuccessors) == 0 else (
        reduce(lambda x, y: x + y, (x[2] for x in activatedBySuccessors)), activatedBySuccessors)

    def get_inhibition_from_conflictors(self, ref_behaviour):
        """
        This method computes the inhibition (actually, activation but negative sign!) caused by other behaviours whose
        preconditions get antagonized when this behaviour runs.
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        inhibitionFromConflictors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == ref_behaviour:  # ignore ourselves
                continue
            for effect in ref_behaviour.correlations:  # this is what we do to sensors
                effect_name = effect.get_pddl_effect_name()  # TODO this has to be reconsidered
                effect_indicator = effect.indicator
                for wish_indicator in self._matching_wishes_indicators(ref_behaviour=behaviour, effect_name=effect_name):
                    # Make a list of all behaviours that have the same bad influence on other behaviours as we have.
                    # Such behaviours are either also negatively correlated another behaviour's wish as we are
                    # or would undo an already satisfied precondition of other behaviours as we would.
                    behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation = [b for b in
                                                                                          self._manager.activeBehaviours
                                                                                          if any(
                            map(lambda x: x * wish_indicator < 0.0 or (x * wish_indicator == 0.0 and x != 0.0),
                                self._matching_effect_indicators(ref_behaviour=b, effect_name=effect_name)))]
                    amount_behaviours_sharing_conflict = len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation) \
                        if len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation) > 0 else 1
                    if wish_indicator * effect_indicator < 0.0:  # if we make an existing conflict stronger
                        # We want the inhibition to be stronger if the condition that we would worsen is almost true.
                        # So we take -(1 - abs(wish * correlation)) as the amount of total inhibition created by this
                        # (for all wishes*effect != 1), in the other case we take a fixed small inhibition of -0.1
                        # conflict and divide it by the number of conflictors
                        if abs(wish_indicator * effect_indicator) == 1:  # without this condition it would result in an inhibition of 0
                            totalInhibition = -0.1  # just take a fixed small inhibition here
                        else:
                            totalInhibition = -(1 - abs(wish_indicator * effect_indicator))
                        totalInhibition = totalInhibition * (behaviour.activation / self._manager.totalActivation) * \
                                          self._conflictor_bias  # TODO somehow strange that we use the conflictor bias here
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating inhibition from conflicted for %s. %s is worsened via %s by %d behaviour(s): %s with a total score of %f",
                                ref_behaviour.name, behaviour.name, effect_name,
                                amount_behaviours_sharing_conflict,
                                behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation, totalInhibition)
                        # Actually only the value is needed but it is a tuple for debug purposes. The length of
                        # behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation says how many more
                        # behaviours cause the same conflict to this conflictor so its inhibition shall be distributed
                        # among them.
                        inhibitionFromConflictors.append((behaviour, effect_name, totalInhibition / amount_behaviours_sharing_conflict))
                    # if we would change the currently existing good state for that behaviour (wish is zero but we have
                    # non-zero correlation to it)
                    elif wish_indicator * effect_indicator == 0.0 and wish_indicator == 0:
                        totalInhibition = -abs(effect_indicator) * (
                        behaviour.activation / self._manager.totalActivation) * self._conflictor_bias
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating inhibition from conflicted for %s. %s is undone via %s (wish: %f) by %d behaviour(s): %s by a total score of %f",
                                ref_behaviour.name, behaviour.name, effect_name, wish_indicator,
                                amount_behaviours_sharing_conflict,
                                behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation, totalInhibition)
                        # The inhibition experienced is my bad influence (my correlation to this effect_name) times the
                        # wish of the other behaviour concerning this effect_name. Actually only the value is needed
                        # but it is a tuple for debug purposes.
                        # amount_behaviours_sharing_conflict knows how many more behaviours cause the same harm to this
                        # conflictor so its inhibition shall be distributed among them.
                        inhibitionFromConflictors.append((behaviour, effect_name, totalInhibition / amount_behaviours_sharing_conflict))
        return (0.0,) if len(inhibitionFromConflictors) == 0 else (
        reduce(lambda x, y: x + y, (x[2] for x in inhibitionFromConflictors)), inhibitionFromConflictors)

    def get_activation_from_plan(self, ref_behaviour):
        """
        This method computes the activation this behaviour receives because of its place on the plan.
        Behaviours at the top of the list will be activated most, other not so much.
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        if not self._manager.plan or ("cost" in self._manager.plan and self._manager.plan["cost"] == -1.0):
            return (0.0, 0xFF)
        own_pddl_name = create_valid_pddl_name(ref_behaviour.name)
        # only consider the plan sequence steps >=current planExecutionIndex
        for index in filter(lambda x: x >= self._manager.planExecutionIndex, sorted(
                self._manager.plan["actions"].keys())):  # walk along the plan starting at where we are
            if self._manager.plan["actions"][index] == own_pddl_name:  # if we are on the plan
                return (1 / (index - self._manager.planExecutionIndex + 1) * self._plan_bias, index)  # index in zero-based
        return (0.0, -1)

    def _matching_effect_indicators(self, ref_behaviour, effect_name):
        """
        returns a list of correlations matching the effect_name.
        :param ref_behaviour: the behaviour with correlations to check against
        :type ref_behaviour: Behaviour
        :param effect_name: effect name
        :type effect_name: str
        :return: lst
        """
        # TODO item.get_pddl_effect_name() might has to be reconsidered, maybe only sensor comparision is sufficient
        # TODO this here can be really problematic as it very much depends on the indicator scale (might have been different with real_world_impact before)
        return [item.indicator for item in ref_behaviour.correlations if item.get_pddl_effect_name() == effect_name]

    def _matching_wishes_indicators(self, ref_behaviour, effect_name):
        """
        returns a list of indicators matching the effect_name.
        :param ref_behaviour: the behaviour with wishes to check against
        :type ref_behaviour: Behaviour
        :param effect_name: effect name
        :type effect_name: str
        :return: lst
        """
        #TODO item.get_pddl_effect_name() might has to be reconsidered, maybe only sensor comparision is sufficient
        return [item.indicator for item in ref_behaviour.wishes if item.get_pddl_effect_name() == effect_name]


ActivationAlgorithmFactory.register_algorithm("default", BaseActivationAlgorithm)


class UniformActivationAlgorithm(BaseActivationAlgorithm):
    """
    This activation algorithm changes the activation calculation formulas in respect to the base algorithm with
    the goal of creating a more uniformly distributed activation result depending on the input values(wish, effect).
    It is also not favouring certain conditions as the base algorithm, e.g. favouring a fulfilled wish over others.
    """

    def __init__(self, manager, extensive_logging=False, create_log_files = False):
        super(UniformActivationAlgorithm, self).__init__(manager, extensive_logging=extensive_logging)

    def get_activation_from_goals(self, ref_behaviour):
        """
        This method computes the activation from goals.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        activated_by_goals = []
        for goal in self._manager.activeGoals:
            # check for each sensor in the goal wishes for behaviours that have sensor effect correlations
            for wish in goal.wishes:
                wish_name = wish.get_pddl_effect_name() #TODO this has to be reconsidered
                wish_indicator = wish.indicator
                # Make a list of all behaviours that are positively correlated to a wish of a goal (those behaviours
                # will get activation from the goal).
                behaviours_activated_by_same_goal = [b for b in self._manager.activeBehaviours if any(
                    map(lambda x: x * wish_indicator > 0.0, self._matching_effect_indicators(ref_behaviour=b, effect_name=wish_name)))]
                amount_activated_behaviours = len(behaviours_activated_by_same_goal) if len(behaviours_activated_by_same_goal) > 0 else 1
                for correlation_indicator in self._matching_effect_indicators(ref_behaviour=ref_behaviour, effect_name=wish_name):
                    # The following condition checks if we affect the sensor in a way that is desirable by the goal
                    if correlation_indicator * wish_indicator > 0.0:
                        total_activation = (correlation_indicator**2 + wish_indicator**2) * self._goal_bias
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating activation from goals for %s. There is/are %d active behaviour(s) that "
                                "support(s) %s via %s: %s with a total activation of %f. GoalBias is %f",
                                ref_behaviour.name, amount_activated_behaviours, goal.name, wish_name,
                                behaviours_activated_by_same_goal, total_activation, self._goal_bias)
                        # The activation we get from that is the product of the correlation we have to this Sensor and
                        # the Goal's desired change of this Sensor. Actually, we are only interested in the value itself
                        # but for debug purposed we make it a tuple including the goal itself
                        activated_by_goals.append((goal, total_activation / amount_activated_behaviours))
        return (0.0,) if len(activated_by_goals) == 0 else (
            reduce(lambda x, y: x + y, (x[1] for x in activated_by_goals)), activated_by_goals)

    def get_inhibition_from_goals(self, ref_behaviour):
        """
        This method computes the inhibition (actually, activation but negative sign!) caused by goals it conflicts with.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        inhibitedByGoals = []
        for goal in self._manager.activeGoals:
            for wish in goal.wishes:
                wish_name = wish.get_pddl_effect_name()  # TODO this has to be reconsidered
                wish_indicator = wish.indicator
                # Make a list of all active behaviours that inhibit this goal.
                # Such behaviours are either negatively correlated to the goals wish (would prevent us from reaching the
                #  goal) or the goal's condition has already been reached and the behaviour would undo it (goal's wish
                # indicator is 0 but there is non-zero correlation of the behaviour to that particular sensor)
                behaviours_inhibited_by_same_goal = [b for b in self._manager.activeBehaviours if any(
                    map(lambda x: x * wish_indicator < 0.0 or (x * wish_indicator == 0.0 and x != 0.0),
                        self._matching_effect_indicators(ref_behaviour=b, effect_name=wish_name)))]
                amount_conflictor_behaviours = len(behaviours_inhibited_by_same_goal) if len(behaviours_inhibited_by_same_goal) > 0 else 1
                for correlation_indicator in self._matching_effect_indicators(ref_behaviour=ref_behaviour, effect_name=wish_name):
                    if correlation_indicator * wish_indicator < 0.0:  # This means we affect the sensor in a way that is not desirable by the goal

                        total_inhibition = -(wish_indicator**2 + correlation_indicator**2) * self._conflictor_bias # TODO somehow strange that we use the conflictor bias here
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating inhibition from goals for %s. There is/are %d behaviours(s) that worsen %s "
                                "via %s: %s and a total inhibition score of %f",
                                ref_behaviour.name, amount_conflictor_behaviours, goal.name, wish_name,
                                behaviours_inhibited_by_same_goal, total_inhibition)
                        # The activation we get from that is the result of the correlation we have to this Sensor and
                        # the Goal's desired change of this Sensor. Note that this is negative, hence the name inhibition!
                        # Actually, we are only interested in the value itself but for debug purposed we make it a tuple
                        # including the goal itself
                        inhibitedByGoals.append((goal, total_inhibition / amount_conflictor_behaviours))
                    elif correlation_indicator != 0 and wish_indicator == 0:  # That means the goals was achieved (wish indicator is 0) but we would undo that (because we are correlated to it somehow)
                        total_inhibition = -correlation_indicator**2 * self._conflictor_bias
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating inhibition from goals for %s. There is/are %d behaviour(s) that undo %s via %s: %s and a total inhibition score of %f",
                                ref_behaviour.name, amount_conflictor_behaviours, goal.name, wish_name,
                                behaviours_inhibited_by_same_goal, total_inhibition)
                        # The activation we get from that is the result of the correlation we have to this Sensor and
                        # the Goal's desired change of this Sensor. Note that this is negative, hence the name inhibition!
                        # Actually, we are only interested in the value itself but for debug purposed we make it a tuple
                        # including the goal itself
                        inhibitedByGoals.append((goal, total_inhibition / amount_conflictor_behaviours))
        return (0.0,) if len(inhibitedByGoals) == 0 else \
            (reduce(lambda x, y: x + y, (x[1] for x in inhibitedByGoals)), inhibitedByGoals)

    def get_activation_from_predecessors(self, ref_behaviour):
        """
        This method computes the activation based on the fact that other behaviours can fulfill a precondition (wish) of this behaviour.
        This is scaled by the "readyness" (precondition satisfaction) of the predecessor as it makes only sense to activate
        a successor if it is likely that it is executable soon.
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        activated_by_predecessors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == ref_behaviour or not behaviour.executable:  # ignore ourselves and non-executable predecessors
                continue
            for wish in ref_behaviour.wishes:  # this is what we wish from a predecessor
                wish_name = wish.get_pddl_effect_name()  # TODO this has to be reconsidered
                wish_indicator = wish.indicator
                # Make a list of all behaviours that share my wish (those will also get activated by the same predecessor). TODO could be improved by just counting --> less memory
                behavioursThatShareThisWish = [b for b in self._manager.activeBehaviours if
                                               any(map(lambda x: x * wish_indicator > 0.0, self._matching_wishes_indicators(ref_behaviour=b, effect_name=wish_name)))]
                amount_wish_sharing_behaviours = len(behavioursThatShareThisWish) if len(behavioursThatShareThisWish) > 0 else 1
                # correlations that the behaviour (potential predecessor) has to the sensor of this wish
                for correlation_indicators in self._matching_effect_indicators(ref_behaviour=behaviour, effect_name=wish_name):
                    # If a predecessor can satisfy my precondition
                    if correlation_indicators * behaviour.preconditionSatisfaction * wish_indicator > 0.0:
                        total_activation = (correlation_indicators**2 + wish_indicator**2) * (
                        behaviour.activation / self._manager.totalActivation) * self._predecessor_bias
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating activation from predecessors for %s. There is/are %d active successor(s) of %s via %s: %s with total activation of %f",
                                ref_behaviour.name, amount_wish_sharing_behaviours, behaviour.name, wish_name,
                                behavioursThatShareThisWish, total_activation)
                        # The activation we get is the likeliness that our predecessor fulfills the preconditions soon.
                        # behavioursThatShareThisWish knows how many more behaviours will get activation from this
                        # predecessor so we distribute it equally
                        activated_by_predecessors.append((behaviour, wish_name, total_activation / amount_wish_sharing_behaviours))
        return (0.0,) if len(activated_by_predecessors) == 0 else \
            (reduce(lambda x, y: x + y, (x[2] for x in activated_by_predecessors)), activated_by_predecessors)

    def get_activation_from_successors(self, ref_behaviour):
        """
        This method computes the activation this behaviour receives because it fulfills a precondition (is a predecessor)
         of a successor which has unfulfilled wishes.
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        activated_by_successors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == ref_behaviour or behaviour.executable:  # ignore ourselves and successors that are already executable
                continue
            for effect in ref_behaviour.correlations:  # this is what can give to a successor
                effect_name = effect.get_pddl_effect_name() # TODO this has to be reconsidered
                effect_indicator = effect.indicator
                # Make a list of all behaviours that are correlated to the same same sensor in the same way as we are. Those are also predecessors like us an get credit from the same successor.
                behaviours_with_same_correlation = [b for b in self._manager.activeBehaviours if any(
                    map(lambda x: x * effect_indicator > 0.0, self._matching_effect_indicators(ref_behaviour=b, effect_name=effect_name)))]
                amount_behaviours_sharing_correlation = len(behaviours_with_same_correlation) if len(behaviours_with_same_correlation) > 0 else 1
                for wish_indicator in self._matching_wishes_indicators(ref_behaviour=behaviour, effect_name=effect_name):  # if we affect other behaviour's wishes somehow
                    if wish_indicator * effect_indicator > 0:  # if we are a predecessor so we get activation from that successor
                        total_activation = (wish_indicator**2 + effect_indicator**2) * (
                        behaviour.activation / self._manager.totalActivation) * self._successor_bias
                        if self._extensive_logging:
                            rhbplog.logdebug("Calculating activation from successors for %s. There is/are %d active "
                                             "predecessor(s) of %s via %s: %s and a total activation score of %f",
                                ref_behaviour.name, amount_behaviours_sharing_correlation, behaviour.name, effect_name,
                                behaviours_with_same_correlation, total_activation)
                        # The activation we get is our expected contribution to the fulfillment of our successors
                        # precondition. Actually only the value is needed but it is a tuple for debug purposes.
                        # amount_behaviours_sharing_correlation is used to distribute activation among all predecessors
                        activated_by_successors.append((behaviour, effect_name, total_activation / amount_behaviours_sharing_correlation))
        return (0.0,) if len(activated_by_successors) == 0 else \
            (reduce(lambda x, y: x + y, (x[2] for x in activated_by_successors)), activated_by_successors)

    def get_inhibition_from_conflictors(self, ref_behaviour):
        """
        This method computes the inhibition (actually, activation but negative sign!) caused by other behaviours whose
        preconditions get antagonized when this behaviour runs.
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        inhibition_from_conflictors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == ref_behaviour:  # ignore ourselves
                continue
            for effect in ref_behaviour.correlations:  # this is what we do to sensors
                effect_name = effect.get_pddl_effect_name()  # TODO this has to be reconsidered
                effect_indicator = effect.indicator
                for wish_indicator in self._matching_wishes_indicators(ref_behaviour=behaviour, effect_name=effect_name):
                    # Make a list of all behaviours that have the same bad influence on other behaviours as we have.
                    # Such behaviours are either also negatively correlated another behaviour's wish as we are
                    # or would undo an already satisfied precondition of other behaviours as we would.
                    behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation = [b for b in
                                                                                          self._manager.activeBehaviours
                                                                                          if any(
                            map(lambda x: x * wish_indicator < 0.0 or (x * wish_indicator == 0.0 and x != 0.0),
                                self._matching_effect_indicators(ref_behaviour=b, effect_name=effect_name)))]
                    amount_behaviours_sharing_conflict = len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation) \
                        if len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation) > 0 else 1
                    if wish_indicator * effect_indicator < 0.0:  # if we make an existing conflict stronger

                        total_inhibition = -(wish_indicator**2 + effect_indicator**2) * \
                                           (behaviour.activation / self._manager.totalActivation) * \
                                           self._conflictor_bias  # TODO somehow strange that we use the conflictor bias here
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating inhibition from conflicted for %s. %s is worsened via %s by %d behaviour(s): %s with a total score of %f",
                                ref_behaviour.name, behaviour.name, effect_name,
                                amount_behaviours_sharing_conflict,
                                behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation, total_inhibition)
                        # Actually only the value is needed but it is a tuple for debug purposes. The length of
                        # behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation says how many more
                        # behaviours cause the same conflict to this conflictor so its inhibition shall be distributed
                        # among them.
                        inhibition_from_conflictors.append((behaviour, effect_name, total_inhibition / amount_behaviours_sharing_conflict))
                    # if we would change the currently existing good state for that behaviour (wish is zero but we have
                    # non-zero correlation to it)
                    elif wish_indicator * effect_indicator == 0.0 and wish_indicator == 0:
                        total_inhibition = -effect_indicator**2 * (behaviour.activation / self._manager.totalActivation)\
                                           * self._conflictor_bias
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating inhibition from conflicted for %s. %s is undone via %s (wish: %f) by %d behaviour(s): %s by a total score of %f",
                                ref_behaviour.name, behaviour.name, effect_name, wish_indicator,
                                amount_behaviours_sharing_conflict,
                                behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation, total_inhibition)
                        # The inhibition experienced is my bad influence (my correlation to this effect_name) times the
                        # wish of the other behaviour concerning this effect_name. Actually only the value is needed
                        # but it is a tuple for debug purposes.
                        # amount_behaviours_sharing_conflict knows how many more behaviours cause the same harm to this
                        # conflictor so its inhibition shall be distributed among them.
                        inhibition_from_conflictors.append((behaviour, effect_name, total_inhibition / amount_behaviours_sharing_conflict))
        return (0.0,) if len(inhibition_from_conflictors) == 0 else \
            (reduce(lambda x, y: x + y, (x[2] for x in inhibition_from_conflictors)), inhibition_from_conflictors)


ActivationAlgorithmFactory.register_algorithm("uniform", UniformActivationAlgorithm)



class ReinforcementLearningActivationAlgorithm(BaseActivationAlgorithm):
    """
    This activation algorithm changes the activation calculation formulas in respect to the base algorithm with
    the goal of creating a more uniformly distributed activation result depending on the input values(wish, effect).
    It is also not favouring certain conditions as the base algorithm, e.g. favouring a fulfilled wish over others.
    """

    def __init__(self, manager, extensive_logging=False, create_log_files = False):
        super(ReinforcementLearningActivationAlgorithm, self).__init__(manager, extensive_logging=extensive_logging)
        self.activation_rl = []
        #self.start_rl_node()
        self.start_rl_class()
        self.SERVICE_TIMEOUT = 5
        self.i = 0
        self.rl_component=None
        self.counter=0
    def start_rl_class(self):
        self.rl_address = self._manager._prefix.replace("/Manager", "_rl_node")
        self.rl_component = RLComponent(name=self.rl_address )

    def start_rl_node(self):
        """
        start the knowledge base node
        """
        package = 'reinforcement_component'
        executable = 'rl_component_node.py'
        self.rl_address=self._manager._prefix.replace("/Manager","_rl_node")
        print(self.rl_address)
        node = roslaunch.core.Node(package=package, node_type=executable, name=self.rl_address,
                                   output='screen')
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        self._rl_process = launch.launch(node)


    def behaviour_to_index(self,name):
        num = 0
        for b in self._manager.behaviours:
            if b == name:
                return num
            num += 1
        return None



    def get_activation_from_rl_node(self):
        """
        :return:num_inputs
                num_outputs
                input_state
                reward
                last_action
        """

        num_outputs = len(self._manager.behaviours)
        if num_outputs == 0:
            print("num outputs cannot be 0")
            return
        input_state = self.transform_input_values()
        num_inputs = input_state.shape[0]
        if num_inputs == 0:
            print("num inputs cannot be 0", input_state)
            return
        reward = self.calculate_reward()
        last_action = self._manager.executedBehaviours
        if len(last_action) == 0:
            print("last action cannot be None")
            return
        last_action_index = self.behaviour_to_index(last_action[0])

        if last_action_index is None:
            print("last action not found")
            return
        # TODO if invalid dim dont send anything just return
        #print(num_inputs,num_outputs,numpy.argmax(input_state),reward,last_action_index)

        input_state_msg = InputState()
        input_state_msg.input_state = input_state
        input_state_msg.num_outputs = num_outputs
        input_state_msg.num_inputs = num_inputs
        input_state_msg.reward = reward
        input_state_msg.last_action = last_action_index

        self.fetchActivation(input_state_msg)


    def fetchActivation(self, msg):
        '''
        This method fetches the status from the actual behaviour node via GetStatus service call
        '''
        self._justFinished_state = False
        try:
            rhbplog.logdebug("Waiting for service %s", self.rl_address + 'GetActivation')
            rospy.wait_for_service(self.rl_address + 'GetActivation', timeout=self.SERVICE_TIMEOUT)
        except rospy.ROSException:
            self._handle_service_timeout()
            return
        try:
            getActivationRequest = rospy.ServiceProxy(self.rl_address + 'GetActivation', GetActivation)
            activation_result = getActivationRequest(msg)
            self.activation_rl = activation_result.activation_state.activations
            #if self._name != condition_state.name:
            #    rhbplog.logerr("%s fetched a status message from a different behaviour: %s. This cannot happen!", self._name, status.name)
            #rhbplog.logdebug("%s reports the following status:\nactivation %s\ncorrelations %s\nprecondition satisfaction %s\n ready threshold %s\nwishes %s\nactive %s\npriority %d\ninterruptable %s",
            #                 self._name, self._activationFromPreconditions, self._correlations, self._preconditionSatisfaction,
            #                 self._readyThreshold, self._wishes, self._active, self._priority, self._interruptable)
        except rospy.ServiceException as e:
            rhbplog.logerr("ROS service exception in 'fetchActivation' of behaviour '%s':", self.rl_address)
            print(e.message)

    def calculate_reward(self):
        """
                this function calculates regarding the fulfillment and priorities of the active goals
                a reward value. it prioritizes the goals in  a way that a higher priority is always more important
                than a smaller one (power of 10)
                :return: 
                """
        # TODO think about better logic maybe
        # active goals or wishes is empty todo fix
        # todo use wishes instead of fulfillment
        reward_value = 0
        for goal in self._manager.activeGoals:
            goal_value = goal.fulfillment * (10 ** goal.priority)
            reward_value += goal_value
        #print("goal",reward_value)
        return reward_value

    def make_hot_state_encoding(self,state,num_state_space):
        state = int(state)
        #print(state,num_state_space)
        array = numpy.identity(num_state_space)[state:state + 1]
        return numpy.identity(num_state_space)[state:state + 1].reshape([num_state_space,1])

    def transform_input_values(self):
        """
        this function uses the wishes and sensors to create the input vectors
        :return: input vector
        """
        # TODO transform like strings or similar to other values . e.g. hot-state-encoding (give sensor choice of encoding)
        # init input array with first row of zeros
        input_array = numpy.zeros([1,1])
        # extend array with input vector from wishes
        for behaviour in self._manager.behaviours:
            # check for each sensor in the goal wishes for behaviours that have sensor effect correlations
            for wish in behaviour.wishes:
                wish_row = numpy.array([wish.indicator]).reshape([1,1])
                input_array = numpy.concatenate([input_array,wish_row])
        # extend array with input vector from sensors
        # save which sensor were already included
        #TODO behavior get input called twice. getstatus
        sensor_input = {}
        for behaviour in self._manager.behaviours:
            for sensor_value in behaviour.sensor_values:
                if not sensor_input.has_key(sensor_value.name):
                    sensor_input[sensor_value.name] = sensor_value.value
                    wish_row = numpy.array([[sensor_value.value]])
                    input_array = numpy.concatenate([input_array, wish_row])
        # cut out first row and return
        # TODO get values of sensors of goal
        for goal in self._manager._goals:
            #print(goal.wishes)
            #print(goal.fulfillment)
            for sensor_value in goal.sensor_values:
                if not sensor_input.has_key(sensor_value.name):
                    #print("sensor vlaue input",sensor_value)
                    value = self.make_hot_state_encoding(sensor_value.value,16)
                    #wish_row = numpy.array([[sensor_value.value]])
                    #value=wish_row
                    sensor_input[sensor_value.name] = value
                    #wish_row = numpy.array([[value]])
                    #print(numpy.array([[5]]))
                    input_array = numpy.concatenate([input_array, value])
        input_array = input_array[1:]
        #print("input:",numpy.argmax(input_array))
        return input_array

    def is_terminal(self, number):
        if number == 5 or number == 7 or number == 11 or number == 12 or number == 15:
            return True
        return False

    def check_if_activation_exists(self,ref_behaviour):


    def compute_behaviour_activation_step(self, ref_behaviour):

        activation_precondition = self.get_activation_from_preconditions(ref_behaviour)
        activation_goals = self.get_activation_from_goals(ref_behaviour)[0]
        inhibition_goals = self.get_inhibition_from_goals(ref_behaviour)[0]
        activation_predecessors = self.get_activation_from_predecessors(ref_behaviour)[0]
        activation_successors = self.get_activation_from_successors(ref_behaviour)[0]
        inhibition_conflictors = self.get_inhibition_from_conflictors(ref_behaviour)[0]
        activation_plan = self.get_activation_from_plan(ref_behaviour)[0]
        # TODO add here the activation from the rl-component
        # TODO e.g. rl_componenet.model.feed_forward()
        # TODO needs reference to rl_component
        # TODO rl-component should save old activation . and a converter for behavior to index
        #self._manager.rl_component.get_model_parameters()
        if self.i % 4 == 0:
            self.get_activation_from_rl_node()
            input = self.transform_input_values()
            if len(input)>0:
                if self.is_terminal(numpy.argmax(input)):
                    self.counter +=1
        self.i+=1
        try:
            this_index=self.behaviour_to_index(ref_behaviour)
            current_activation_step = self.activation_rl[this_index]*10000

            self.epsilon = 1. / ((self.counter / 50) + 10)
            if numpy.random.rand(1) < self.epsilon:
                print("choose random action",ref_behaviour)
                current_activation_step = 1000000.0
            ref_behaviour.current_activation_step = 0
            ref_behaviour.activation = current_activation_step
            #print(ref_behaviour,this_index, current_activation_step ,ref_behaviour.activation)
            #self.rl_component.update_model()
            return current_activation_step
        except Exception as e:
            print(e.message)
        #self.activation_rl[self.behaviour_to_index(ref_behaviour)]
        # TODO implement here logic for random execution of behaviors in greedely manner for exploration.

        rhbplog.loginfo("\t%s: activation from preconditions: %s", ref_behaviour, activation_precondition)
        rhbplog.loginfo("\t%s: activation from goals: %s", ref_behaviour, activation_goals)
        rhbplog.loginfo("\t%s: inhibition from goals: %s", ref_behaviour, inhibition_goals)
        rhbplog.loginfo("\t%s: activation from predecessors: %s", ref_behaviour, activation_predecessors)
        rhbplog.loginfo("\t%s: activation from successors: %s", ref_behaviour, activation_successors)
        rhbplog.loginfo("\t%s: inhibition from conflicted: %s", ref_behaviour, inhibition_conflictors)
        rhbplog.loginfo("\t%s: activation from plan: %s", ref_behaviour, activation_plan)

        current_activation_step = activation_precondition \
                                  + activation_goals \
                                  + inhibition_goals \
                                  + activation_predecessors \
                                  + activation_successors \
                                  + inhibition_conflictors \
                                  + activation_plan

        ref_behaviour.current_activation_step = current_activation_step

        return current_activation_step


ActivationAlgorithmFactory.register_algorithm("reinforcement", ReinforcementLearningActivationAlgorithm)
