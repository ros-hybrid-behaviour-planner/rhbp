from decomposition_components.cost_computing import PDDLCostEvaluator
from decomposition_components.delegation_clients import RHBPDelegationClient


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
            prefix = self.__behaviour_manager.prefix
            self.add_own_cost_evaluator(cost_evaluator=new_cost_evaluator, manager_name=prefix)
            self._delegation_manager.start_depth_service(prefix=prefix)
            self._added_cost_evaluator = True

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