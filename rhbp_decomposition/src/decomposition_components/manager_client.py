from decomposition_components.cost_computing import PDDLCostEvaluator
from decomposition_components.delegation_clients import RHBPDelegationClient
from delegation_components.delegation_manager import DelegationManager


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
        self.__behaviour_manager = manager
        self._added_cost_evaluator = False

    def make_cost_computable(self):
        if self._added_cost_evaluator:
            return

        if not self._active_manager or self._delegation_manager.cost_computable:
            self._delegation_manager = DelegationManager(instance_name=self.__behaviour_manager.prefix, max_tasks=20)

        new_cost_evaluator = self.get_new_cost_evaluator()
        prefix = self.__behaviour_manager.prefix
        self.add_own_cost_evaluator(cost_evaluator=new_cost_evaluator, manager_name=prefix)
        self._delegation_manager.start_depth_service(prefix=prefix)
        self._added_cost_evaluator = True

    def remove_cost_computable(self):
        if self._added_cost_evaluator:
            self._delegation_manager.remove_cost_function_evaluator()
            self._added_cost_evaluator = False

    def unregister(self):
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

        new_cost_evaluator = PDDLCostEvaluator(manager=self.__behaviour_manager)

        return new_cost_evaluator