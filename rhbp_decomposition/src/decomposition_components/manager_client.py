"""
Special kind of DelegationClient used by RHBP Managers

@author: Mengers
"""

from decomposition_components.cost_computing import PDDLCostEvaluator
from decomposition_components.delegation_clients import RHBPDelegationClient
from delegation_components.delegation_manager import DelegationManager
from copy import copy


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

        super(RHBPManagerDelegationClient, self).__init__(checking_prefix=manager.prefix)
        self._rhbp_manager = manager
        self._added_cost_evaluator = False
        self._actively_pursued_goals = []

    def make_cost_computable(self):
        """
        Makes sure that a cost evaluator for this manager will be added to the
        DelegationManager. If that DelegationManager is already taken, a new one
        will be created
        """

        if self._added_cost_evaluator:
            return

        if not self._active_manager or self._delegation_manager.cost_computable:
            self._create_own_delegation_manager()

        new_cost_evaluator = self.get_new_cost_evaluator()
        prefix = self._rhbp_manager.prefix
        self.add_own_cost_evaluator(cost_evaluator=new_cost_evaluator, agent_name=prefix)
        self._delegation_manager.start_depth_service(prefix=prefix)
        self._added_cost_evaluator = True

    def _create_own_delegation_manager(self):
        """
        Creates a new DelegationManager just for this Client
        """

        self._delegation_manager = DelegationManager(name=self._rhbp_manager.prefix)

    def remove_cost_computable(self):
        """
        Removes the CostEvaluator that this client constructed from the
        DelegationManager
        """

        if self._added_cost_evaluator:
            self._delegation_manager.remove_cost_function_evaluator()
            self._added_cost_evaluator = False

    def unregister(self):
        """
        Unregisters this Client from the DelegationManager and removes the
        CostEvaluator
        """

        if self._active_manager and self._added_cost_evaluator:
            self._delegation_manager.stop_depth_service()
            self._delegation_manager.remove_cost_function_evaluator()
            self._added_cost_evaluator = False
        super(RHBPManagerDelegationClient, self).unregister()

    def get_new_cost_evaluator(self):
        """
        Constructs a new cost_evaluator and returns it

        :return: a cost_evaluator using the managers planning functions
        :rtype: PDDLCostEvaluator
        """

        new_cost_evaluator = PDDLCostEvaluator(manager=self._rhbp_manager)

        return new_cost_evaluator

    def update_actively_pursued_goals(self, current_active_goals):
        """
        Updates currently pursued goals to notify the DelegationManager about
        goals that are not longer pursued

        :param current_active_goals: list of the goals that are currently
                pursued by the corresponding manager
        :type current_active_goals: list(goal)
        """

        if self._active_manager and self._added_cost_evaluator:
            # for all goals that are not currently pursued, the goal
            # could correspond to a task, which is failed now
            for goal in self._actively_pursued_goals:
                if not current_active_goals.__contains__(goal):
                    self._delegation_manager.fail_task(goal_name=goal.name)

        self._actively_pursued_goals = copy(current_active_goals)
