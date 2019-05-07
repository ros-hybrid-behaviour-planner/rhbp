'''
Created on 22.03.2017

@author: hrabia, wypler
'''
from __future__ import division  # force floating point division when using plain /
from abc import ABCMeta, abstractmethod

# from .managers import Manager has to be commented because of circular dependency problem

from math import sqrt

from .behaviours import Behaviour
from .pddl import create_valid_pddl_name
from rhbp_core.msg import Activation
import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.planning')


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
        self._extensive_logging = extensive_logging

    def step_preparation(self):
        """
        this callback can optionally be overwritten to run preparation for a step before all behaviours are processed
        """
        pass

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
    def update_config(self, **kwargs):
        """
        Method for updating the algorithm configuration with named parameters
        :param kwargs:
        """
        pass

    @abstractmethod
    def is_planner_enabled(self):
        """
        Return if the planner is enabled in the current configuration
        :return: True/False
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

    def __init__(self, manager, extensive_logging=False, create_log_files=False):
        super(BaseActivationAlgorithm, self).__init__(manager, extensive_logging=extensive_logging)

        # some defaults but are usually overwritten directly with update_config
        self._situation_bias = 1.0
        self._plan_bias = 1.0
        self._conflictor_bias = 1.0
        self._goal_bias = 1.0
        self._successor_bias = 1.0
        self._predecessor_bias = 1.0
        self._activation_decay = 0.9
        self._goal_priority_weights = {}
        self._apply_goal_priority_weights = False

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
        self._situation_bias = kwargs.get('situationBias', self._situation_bias)
        self._plan_bias = kwargs.get('planBias', self._plan_bias)
        self._conflictor_bias = kwargs.get('conflictorBias', self._conflictor_bias)
        self._goal_bias = kwargs.get('goalBias', self._goal_bias)
        self._successor_bias = kwargs.get('successorBias', self._successor_bias)
        self._predecessor_bias = kwargs.get('predecessorBias', self._predecessor_bias)
        self._activation_decay = kwargs.get('activationDecay', self._activation_decay)
        self._apply_goal_priority_weights = kwargs.get('activationGoalPriority', self._apply_goal_priority_weights)

        rhbplog.loginfo("Activation weights updated: \n"
                        "situation_bias:%f\n"
                        "plan_bias:%f\n"
                        "goal_bias:%f\n"
                        "with_goal_priority_weights:%s\n"
                        "successor_bias:%f\n"
                        "predecessor_bias:%f\n"
                        "conflictor_bias:%f\n"
                        "activation_decay:%f\n",
                        self._situation_bias, self._plan_bias, self._goal_bias, self._apply_goal_priority_weights,
                        self._successor_bias, self._predecessor_bias, self._conflictor_bias, self._activation_decay
                        )

    def is_planner_enabled(self):
        return self._plan_bias > 0.0

    def step_preparation(self):

        if self._apply_goal_priority_weights:

            self._goal_priority_weights = self._calculate_goal_priority_weights()
            rhbplog.logdebug("Goal prio weights: %s", str(self._goal_priority_weights))

    def _calculate_goal_priority_weights(self, min_weight=1, max_weight=2):
        """
        calculate weights considering the priority of goals
        :return: dict {goal:prio_weight}
        """

        goal_weights = {}

        if self._manager.operational_goals:

            max_prio = max(self._manager.operational_goals, key=lambda g: g.priority).priority
            min_prio = min(self._manager.operational_goals, key=lambda g: g.priority).priority

            range_prio = max_prio-min_prio

            if range_prio == 0:  # all goals have the same priority
                for goal in self._manager.operational_goals:
                    goal_weights[goal] = 1.0
            else:
                weight_range = max_weight - min_weight  # target range between 1 and 2

                a = weight_range / range_prio
                b = max_weight - a * max_prio

                for goal in self._manager.operational_goals:
                    goal_weights[goal] = goal.priority * a + b

        return goal_weights

    def compute_behaviour_activation_step(self, ref_behaviour):

        activation_precondition = self.get_activation_from_preconditions(ref_behaviour)
        activation_goals = self.get_activation_from_goals(ref_behaviour)[0]
        inhibition_goals = self.get_inhibition_from_goals(ref_behaviour)[0]
        activation_predecessors = self.get_activation_from_predecessors(ref_behaviour)[0]
        activation_successors = self.get_activation_from_successors(ref_behaviour)[0]
        inhibition_conflictors = self.get_inhibition_from_conflictors(ref_behaviour)[0]
        activation_plan = self.get_activation_from_plan(ref_behaviour)[0]

        rhbplog.loginfo("\t%s: activation from preconditions: %2.3f", ref_behaviour, activation_precondition)
        rhbplog.loginfo("\t%s: activation from goals: %2.3f", ref_behaviour, activation_goals)
        rhbplog.loginfo("\t%s: inhibition from goals: %2.3f", ref_behaviour, inhibition_goals)
        rhbplog.loginfo("\t%s: activation from predecessors: %2.3f", ref_behaviour, activation_predecessors)
        rhbplog.loginfo("\t%s: activation from successors: %2.3f", ref_behaviour, activation_successors)
        rhbplog.loginfo("\t%s: inhibition from conflicted: %2.3f", ref_behaviour, inhibition_conflictors)
        rhbplog.loginfo("\t%s: activation from plan: %2.3f", ref_behaviour, activation_plan)

        current_activation_step = activation_precondition \
                                  + activation_goals \
                                  + inhibition_goals \
                                  + activation_predecessors \
                                  + activation_successors \
                                  + inhibition_conflictors \
                                  + activation_plan

        ref_behaviour.current_activation_step = current_activation_step
        ref_behaviour.activation_components = [Activation('Precondition', activation_precondition),
                                               Activation('Goal', activation_goals),
                                               Activation('Goal-Inhibition', inhibition_goals),
                                               Activation('Predecessors', activation_predecessors),
                                               Activation('Successors', activation_successors),
                                               Activation('Conflictors', inhibition_conflictors),
                                               Activation('Plan', activation_plan),
                                               ]

        return current_activation_step

    def commit_behaviour_activation(self, ref_behaviour):
        activation = ref_behaviour.activation * self._activation_decay + ref_behaviour.current_activation_step
        if activation < 0.0:
            activation = 0.0
        ref_behaviour.update_activation(activation=activation, step=self._manager.current_step)
        ref_behaviour.current_activation_step = 0.0

        return activation

    def get_activation_from_preconditions(self, ref_behaviour):
        """
        This methods computes the activation from the behaviour preconditions
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        return ref_behaviour.activationFromPreconditions * self._situation_bias

    def get_activation_from_goals(self, ref_behaviour):
        """
        This method computes the activation from goals.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        activatedByGoals = []
        for goal in self._manager.operational_goals:
            if self._apply_goal_priority_weights:
                prio_weight = self._goal_priority_weights.get(goal, 1.0)
            else:
                prio_weight = 1.0

            # check for each sensor in the goal wishes for behaviours that have sensor effect correlations
            for wish in goal.wishes:
                wish_name = wish.get_pddl_effect_name()
                wish_indicator = wish.indicator
                # Make a list of all behaviours that are positively correlated to a wish of a goal (those behaviours will get activation from the goal).
                behavioursActivatedBySameGoal = [b for b in self._manager.operational_behaviours if any(
                    map(lambda x: x * wish_indicator > 0.0, self._matching_effect_indicators(ref_behaviour=b, effect_name=wish_name)))]
                amount_activated_behaviours = len(behavioursActivatedBySameGoal) if len(behavioursActivatedBySameGoal) > 0 else 1
                for correlation_indicator in self._matching_effect_indicators(ref_behaviour=ref_behaviour, effect_name=wish_name):
                    if correlation_indicator * wish_indicator > 0.0:  # This means we affect the sensor in a way that is desirable by the goal

                        totalActivation = correlation_indicator * wish_indicator * prio_weight * self._goal_bias
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
        for goal in self._manager.operational_goals:

            if self._apply_goal_priority_weights:
                prio_weight = self._goal_priority_weights.get(goal, 1.0)
            else:
                prio_weight = 1.0

            for wish in goal.wishes:
                wish_name = wish.get_pddl_effect_name()
                wish_indicator = wish.indicator
                # Make a list of all active behaviours that inhibit this goal.
                # Such behaviours are either negatively correlated to the goals wish (would prevent us from reaching the goal)
                # or the goal's condition has already been reached and the behaviour would undo it (goal's wish indicator is 0 but there is non-zero correlation of the behaviour to that particular sensor)
                behavioursInhibitedBySameGoal = [b for b in self._manager.operational_behaviours if any(
                    map(lambda x: x * wish_indicator < 0.0 or (x * wish_indicator == 0.0 and x != 0.0),
                        self._matching_effect_indicators(ref_behaviour=b, effect_name=wish_name)))]
                amount_conflictor_behaviours = len(behavioursInhibitedBySameGoal) if len(
                    behavioursInhibitedBySameGoal) > 0 else 1
                for correlation_indicator in self._matching_effect_indicators(ref_behaviour=ref_behaviour,
                                                                              effect_name=wish_name):
                    if correlation_indicator * wish_indicator < 0.0:  # This means we affect the sensor in a way that is not desirable by the goal
                        # We want the inhibition to be stronger if the condition that we would worsen is almost true.
                        # So we take -(1 - abs(wish * correlation)) as the amount of total inhibition created by this
                        # (for all wishes*effect != 1), in the other case we take a fixed small inhibition of -0.1
                        # conflict and divide it by the number of conflictors
                        if (abs(
                                    wish_indicator * correlation_indicator)) == 1:  # without this condition it would result in an inhibition of 0
                            totalInhibition = -0.1  # just take a fixed small inhibition here
                        else:
                            totalInhibition = -(1 - abs(wish_indicator * correlation_indicator))

                        # TODO somehow strange that we use the conflictor bias here
                        totalInhibition = totalInhibition * prio_weight * self._conflictor_bias
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
                        totalInhibition = -abs(correlation_indicator) * self._conflictor_bias * prio_weight
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
        for behaviour in self._manager.operational_behaviours:
            if behaviour == ref_behaviour or not behaviour.executable:  # ignore ourselves and non-executable predecessors
                continue
            for wish in ref_behaviour.wishes:  # this is what we wish from a predecessor
                wish_name = wish.get_pddl_effect_name()
                wish_indicator = wish.indicator
                # Make a list of all behaviours that share my wish (those will also get activated by the same predecessor). TODO could be improved by just counting --> less memory
                behavioursThatShareThisWish = [b for b in self._manager.operational_behaviours if
                                               any(map(lambda x: x * wish_indicator > 0.0, self._matching_wishes_indicators(ref_behaviour=b, effect_name=wish_name)))]
                amount_wish_sharing_behaviours = len(behavioursThatShareThisWish) if len(behavioursThatShareThisWish) > 0 else 1
                # correlations that the behaviour (potential predecessor) has to the sensor of this wish
                for correlation_indicators in self._matching_effect_indicators(ref_behaviour=behaviour,
                                                                               effect_name=wish_name):
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
                        activatedByPredecessors.append(
                            (behaviour, wish_name, totalActivation / amount_wish_sharing_behaviours))
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
        for behaviour in self._manager.operational_behaviours:
            if behaviour == ref_behaviour or behaviour.executable:  # ignore ourselves and successors that are already executable
                continue
            for effect in ref_behaviour.correlations:  # this is what can give to a successor
                effect_name = effect.get_pddl_effect_name()
                effect_indicator = effect.indicator
                # Make a list of all behaviours that are correlated to the same same sensor in the same way as we are. Those are also predecessors like us an get credit from the same successor.
                behavioursThatShareOurCorrelation = [b for b in self._manager.operational_behaviours if any(
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
                        activatedBySuccessors.append(
                            (behaviour, effect_name, totalActivation / amount_behaviours_sharing_correlation))
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
        for behaviour in self._manager.operational_behaviours:
            if behaviour == ref_behaviour:  # ignore ourselves
                continue
            for effect in ref_behaviour.correlations:  # this is what we do to sensors
                effect_name = effect.get_pddl_effect_name()
                effect_indicator = effect.indicator
                for wish_indicator in self._matching_wishes_indicators(ref_behaviour=behaviour,
                                                                       effect_name=effect_name):
                    # Make a list of all behaviours that have the same bad influence on other behaviours as we have.
                    # Such behaviours are either also negatively correlated another behaviour's wish as we are
                    # or would undo an already satisfied precondition of other behaviours as we would.
                    behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation = [b for b in
                                                                                          self._manager.operational_behaviours
                                                                                          if any(
                            map(lambda x: x * wish_indicator < 0.0 or (x * wish_indicator == 0.0 and x != 0.0),
                                self._matching_effect_indicators(ref_behaviour=b, effect_name=effect_name)))]
                    amount_behaviours_sharing_conflict = len(
                        behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation) \
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
                        inhibitionFromConflictors.append(
                            (behaviour, effect_name, totalInhibition / amount_behaviours_sharing_conflict))
                    # if we would change the currently existing good state for that behaviour (wish is zero but we have
                    # non-zero correlation to it)
                    elif wish_indicator == 0:
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
                        inhibitionFromConflictors.append(
                            (behaviour, effect_name, totalInhibition / amount_behaviours_sharing_conflict))
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
                return (
                    1 / (index - self._manager.planExecutionIndex + 1) * self._plan_bias, index)  # index in zero-based
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
        # TODO item.get_pddl_effect_name() might has to be reconsidered, maybe only sensor comparision is sufficient
        return [item.indicator for item in ref_behaviour.wishes if item.get_pddl_effect_name() == effect_name]


ActivationAlgorithmFactory.register_algorithm("default", BaseActivationAlgorithm)


class UniformActivationAlgorithm(BaseActivationAlgorithm):
    """
    This activation algorithm changes the activation calculation formulas in respect to the base algorithm with
    the goal of creating a more uniformly distributed activation result depending on the input values(wish, effect).
    It is also not favouring certain conditions as the base algorithm, e.g. favouring a fulfilled wish over others.
    """

    def __init__(self, manager, extensive_logging=False, create_log_files=False):
        super(UniformActivationAlgorithm, self).__init__(manager, extensive_logging=extensive_logging)

    def get_activation_from_goals(self, ref_behaviour):
        """
        This method computes the activation from goals.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        :param ref_behaviour: the behaviour for which the activation is determined
        :type ref_behaviour: Behaviour
        """
        activated_by_goals = []
        for goal in self._manager.operational_goals:

            if self._apply_goal_priority_weights:
                prio_weight = self._goal_priority_weights.get(goal, 1.0)
            else:
                prio_weight = 1.0

            # check for each sensor in the goal wishes for behaviours that have sensor effect correlations
            for wish in goal.wishes:
                wish_name = wish.get_pddl_effect_name()
                wish_indicator = wish.indicator
                # Make a list of all behaviours that are positively correlated to a wish of a goal (those behaviours
                # will get activation from the goal).
                behaviours_activated_by_same_goal = [b for b in self._manager.operational_behaviours if any(
                    map(lambda x: x * wish_indicator > 0.0, self._matching_effect_indicators(ref_behaviour=b, effect_name=wish_name)))]
                amount_activated_behaviours = len(behaviours_activated_by_same_goal) if len(behaviours_activated_by_same_goal) > 0 else 1
                for correlation_indicator in self._matching_effect_indicators(ref_behaviour=ref_behaviour, effect_name=wish_name):
                    # The following condition checks if we affect the sensor in a way that is desirable by the goal
                    if correlation_indicator * wish_indicator > 0.0:
                        total_activation = sqrt(correlation_indicator ** 2 + wish_indicator ** 2) * prio_weight \
                                           * self._goal_bias
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating activation from goals for %s. There is/are %d operational behaviour(s) that"
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
        for goal in self._manager.operational_goals:

            if self._apply_goal_priority_weights:
                prio_weight = self._goal_priority_weights.get(goal, 1.0)
            else:
                prio_weight = 1.0

            for wish in goal.wishes:
                wish_name = wish.get_pddl_effect_name()
                wish_indicator = wish.indicator
                # Make a list of all active behaviours that inhibit this goal.
                # Such behaviours are either negatively correlated to the goals wish (would prevent us from reaching the
                #  goal) or the goal's condition has already been reached and the behaviour would undo it (goal's wish
                # indicator is 0 but there is non-zero correlation of the behaviour to that particular sensor)
                behaviours_inhibited_by_same_goal = [b for b in self._manager.operational_behaviours if any(
                    map(lambda x: x * wish_indicator < 0.0 or (x * wish_indicator == 0.0 and x != 0.0),
                        self._matching_effect_indicators(ref_behaviour=b, effect_name=wish_name)))]
                amount_conflictor_behaviours = len(behaviours_inhibited_by_same_goal) if len(
                    behaviours_inhibited_by_same_goal) > 0 else 1
                for correlation_indicator in self._matching_effect_indicators(ref_behaviour=ref_behaviour,
                                                                              effect_name=wish_name):
                    # First term we affect the sensor in a way that is not desirable by the goal
                    # Second term means the goals was achieved (wish indicator is 0) but we potentially undo that
                    # (because we are correlated to it somehow)
                    if correlation_indicator * wish_indicator < 0.0 or \
                       correlation_indicator != 0 and wish_indicator == 0:

                        total_inhibition = -sqrt(wish_indicator ** 2 + correlation_indicator ** 2) * prio_weight \
                                           * self._conflictor_bias  # TODO somehow strange that we use the conflictor bias here
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
        for behaviour in self._manager.operational_behaviours:
            if behaviour == ref_behaviour or not behaviour.executable:  # ignore ourselves and non-executable predecessors
                continue
            for wish in ref_behaviour.wishes:  # this is what we wish from a predecessor
                wish_name = wish.get_pddl_effect_name()
                wish_indicator = wish.indicator
                # Make a list of all behaviours that share my wish (those will also get activated by the same predecessor). TODO could be improved by just counting --> less memory
                behavioursThatShareThisWish = [b for b in self._manager.operational_behaviours if
                                               any(map(lambda x: x * wish_indicator > 0.0, self._matching_wishes_indicators(ref_behaviour=b, effect_name=wish_name)))]
                amount_wish_sharing_behaviours = len(behavioursThatShareThisWish) if len(behavioursThatShareThisWish) > 0 else 1
                # correlations that the behaviour (potential predecessor) has to the sensor of this wish
                for correlation_indicators in self._matching_effect_indicators(ref_behaviour=behaviour,
                                                                               effect_name=wish_name):
                    # If a predecessor can satisfy my precondition
                    if correlation_indicators * behaviour.preconditionSatisfaction * wish_indicator > 0.0:
                        total_activation = sqrt(correlation_indicators ** 2 + wish_indicator ** 2) * (
                            behaviour.activation / self._manager.totalActivation) * self._predecessor_bias
                        if self._extensive_logging:
                            rhbplog.logdebug(
                                "Calculating activation from predecessors for %s. There is/are %d active successor(s) of %s via %s: %s with total activation of %f",
                                ref_behaviour.name, amount_wish_sharing_behaviours, behaviour.name, wish_name,
                                behavioursThatShareThisWish, total_activation)
                        # The activation we get is the likeliness that our predecessor fulfills the preconditions soon.
                        # behavioursThatShareThisWish knows how many more behaviours will get activation from this
                        # predecessor so we distribute it equally
                        activated_by_predecessors.append(
                            (behaviour, wish_name, total_activation / amount_wish_sharing_behaviours))
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
        for behaviour in self._manager.operational_behaviours:
            if behaviour == ref_behaviour or behaviour.executable:  # ignore ourselves and successors that are already executable
                continue
            for effect in ref_behaviour.correlations:  # this is what can give to a successor
                effect_name = effect.get_pddl_effect_name()
                effect_indicator = effect.indicator
                # Make a list of all behaviours that are correlated to the same same sensor in the same way as we are. Those are also predecessors like us an get credit from the same successor.
                behaviours_with_same_correlation = [b for b in self._manager.operational_behaviours if any(
                    map(lambda x: x * effect_indicator > 0.0, self._matching_effect_indicators(ref_behaviour=b, effect_name=effect_name)))]
                amount_behaviours_sharing_correlation = len(behaviours_with_same_correlation) if len(behaviours_with_same_correlation) > 0 else 1
                for wish_indicator in self._matching_wishes_indicators(ref_behaviour=behaviour, effect_name=effect_name):  # if we affect other behaviour's wishes somehow
                    if wish_indicator * effect_indicator > 0:  # if we are a predecessor so we get activation from that successor
                        total_activation = sqrt(wish_indicator ** 2 + effect_indicator ** 2) * (
                            behaviour.activation / self._manager.totalActivation) * self._successor_bias
                        if self._extensive_logging:
                            rhbplog.logdebug("Calculating activation from successors for %s. There is/are %d active "
                                             "predecessor(s) of %s via %s: %s and a total activation score of %f",
                                             ref_behaviour.name, amount_behaviours_sharing_correlation, behaviour.name,
                                             effect_name,
                                             behaviours_with_same_correlation, total_activation)
                        # The activation we get is our expected contribution to the fulfillment of our successors
                        # precondition. Actually only the value is needed but it is a tuple for debug purposes.
                        # amount_behaviours_sharing_correlation is used to distribute activation among all predecessors
                        activated_by_successors.append(
                            (behaviour, effect_name, total_activation / amount_behaviours_sharing_correlation))
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
        for behaviour in self._manager.operational_behaviours:
            if behaviour == ref_behaviour:  # ignore ourselves
                continue
            for effect in ref_behaviour.correlations:  # this is what we do to sensors
                effect_name = effect.get_pddl_effect_name()
                effect_indicator = effect.indicator
                for wish_indicator in self._matching_wishes_indicators(ref_behaviour=behaviour,
                                                                       effect_name=effect_name):
                    # Make a list of all behaviours that have the same bad influence on other behaviours as we have.
                    # Such behaviours are either also negatively correlated another behaviour's wish as we are
                    # or would undo an already satisfied precondition of other behaviours as we would.
                    behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation = [b for b in
                                                                                          self._manager.operational_behaviours
                                                                                          if any(
                            map(lambda x: x * wish_indicator < 0.0 or (x * wish_indicator == 0.0 and x != 0.0),
                                self._matching_effect_indicators(ref_behaviour=b, effect_name=effect_name)))]
                    amount_behaviours_sharing_conflict = len(
                        behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation) \
                        if len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation) > 0 else 1

                    # first term: if we make an existing conflict stronger
                    # second term: if we would change the currently existing good state for that behaviour
                    #              (wish is zero but we have non-zero correlation to it)
                    if wish_indicator * effect_indicator < 0.0 \
                            or wish_indicator * effect_indicator == 0.0 and wish_indicator == 0:

                        total_inhibition = - sqrt(wish_indicator ** 2 + effect_indicator ** 2) * \
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
                        inhibition_from_conflictors.append(
                            (behaviour, effect_name, total_inhibition / amount_behaviours_sharing_conflict))

        return (0.0,) if len(inhibition_from_conflictors) == 0 else \
            (reduce(lambda x, y: x + y, (x[2] for x in inhibition_from_conflictors)), inhibition_from_conflictors)


ActivationAlgorithmFactory.register_algorithm("uniform", UniformActivationAlgorithm)
