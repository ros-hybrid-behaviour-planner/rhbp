
from delegation_components.delegation_clients import DelegationClientBase

from delegation_components.goal_wrapper import RHBPGoalWrapper
from delegation_components.cost_computing import PDDLCostEvaluator


import utils.rhbp_logging
rhbplogger = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.delegation')


class RHBPDelegationClient(DelegationClientBase):
    """
    DelegationClient for the RHBP if no tasks can be taken and no cost be
    evaluated
    """

    logger = rhbplogger

    def delegate(self, goal_name, conditions=None, satisfaction_threshold=1.0):
        """
        Tries to delegate a goal with given parameters

        :param goal_name: name of the goal
        :type goal_name: str
        :param conditions: a list of conditions
        :type conditions: list
        :param satisfaction_threshold: the satisfaction threshold of the goal
        :type satisfaction_threshold: float
        :return: ID of the delegation
        :rtype: int
        :raises RuntimeError: if no DelegationManager is registered
        """

        if not self._active_manager:
            raise RuntimeError("Delegation without a registered DelegationManager")

        if conditions is None:
            self.logger.logwarn("Trying to delegate a goal without conditions")
            conditions = []

        condition_string = "\n\t".join([str(x) for x in conditions])
        self.logger.loginfo("New delegation attempt with the conditions:\n\t" + condition_string + "\n\t and the satisfaction threshold of " + str(satisfaction_threshold))

        new_goal_wrapper = RHBPGoalWrapper(name=goal_name, conditions=conditions, satisfaction_threshold=satisfaction_threshold)

        delegation_id = self.delegate_goal_wrapper(goal_wrapper=new_goal_wrapper)

        return delegation_id


class RHBPManagerDelegationClient(RHBPDelegationClient):
    """
    Version of the RHBPDelegationClient used for Managers that handle goals as
    tasks and cost evaluation.
    """

    def __init__(self, manager):
        """
        Constructor for the client

        :param manager: a Manager from RHBP
        :type manager: Manager
        """

        super(RHBPManagerDelegationClient, self).__init__()
        self.__behaviour_manager = manager

    def register(self, delegation_manager, add_own_cost_evaluator=True):
        """
        Registers a delegation_manager at this client and adds a
        cost_function_evaluator to him, if wanted

        :param delegation_manager: DelegationManager from task_decomposition
                module
        :type delegation_manager: DelegationManager
        :param add_own_cost_evaluator: determines if a cost_function_evaluator
                that is using the instance of the connected Manager should be
                added to the DelegationManager, mainly important for scenarios
                with a DelegationManager instance for multiple Managers
        :type add_own_cost_evaluator: bool
        """

        if self._active_manager:
            self.logger.logwarn("Attempt to log a new delegation_manager with the name \"" + str(delegation_manager.get_name())
                                + "\" while one with the name \"" + str(self._delegation_manager.get_name()) + "\" is already registered.\nNew DelegationManager will be ignored.")
            # will still use the old registered one
            return

        super(RHBPManagerDelegationClient, self).register(delegation_manager=delegation_manager)

        if add_own_cost_evaluator:
            new_cost_evaluator = self.get_new_cost_evaluator()
            self.add_own_cost_evaluator(cost_evaluator=new_cost_evaluator, manager_name=self.__behaviour_manager._prefix)

    def get_new_cost_evaluator(self):
        """
        Constructs a new cost_evaluator and returns it

        :return: a cost_evaluator using the managers planning functions
        :rtype: PDDLCostEvaluator
        """

        new_cost_evaluator = PDDLCostEvaluator(planning_function=self.__behaviour_manager.plan_with_additional_goal)

        return new_cost_evaluator
