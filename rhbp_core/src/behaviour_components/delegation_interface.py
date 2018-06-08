

from delegation_components.goalwrapper import RHBPGoalWrapper
from delegation_components.cost_computing import PDDLCostEvaluator

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.delegation')


class DelegationInterface(object):

    def __init__(self, manager):
        self.__behaviour_manager = manager
        self.__delegation_manager = None
        self.__active_manager = False

    def get_cost_evaluator(self):

        new_cost_evaluator = PDDLCostEvaluator(planning_function=self.__behaviour_manager.plan_with_additional_goal)

        return new_cost_evaluator

    def delegate(self, name, conditions, satisfaction_threshold):
        if not self.__active_manager:
            # TODO do we want to return bools or raise exceptions
            return

        condition_string = "\n\t".join([str(x) for x in conditions])
        rhbplog.loginfo("New delegation attempt with the conditions:\n\t" + condition_string + "\n\t and the satisfaction threshold of " + str(satisfaction_threshold))

        new_goal_wrapper = RHBPGoalWrapper(name=name, conditions=conditions, satisfaction_threshold=satisfaction_threshold)

        self.__delegation_manager.delegate(goal_wrapper=new_goal_wrapper)
        # TODO return value

    def register(self, delegation_manager):

        if self.__active_manager:
            rhbplog.logwarn("Attempt to log a new delegation_manager with the name \"" + str(delegation_manager.get_manager_name()) + "\" while one with the name \"" + str(self.__delegation_manager.get_manager_name()) + "\" is already registered")
            # TODO raise exception or not?
            return

        self.__delegation_manager = delegation_manager
        self.__active_manager = True

    def unregister(self):

        self.__active_manager = False
        self.__delegation_manager = None
