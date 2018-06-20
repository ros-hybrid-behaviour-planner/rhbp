

from delegation_components.goalwrapper import RHBPGoalWrapper
from delegation_components.cost_computing import PDDLCostEvaluator


import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.delegation')


class DelegationClient(object):
    """
    Client for the DelegationManager, part of the task decomposition module
    that directly interfaces with outside objects.
    """

    instance_counter = 0
    all_clients_and_ids = {}

    @classmethod
    def unregister_at(cls, client_ids):
        """
        Unregisters the DelegationManager at all clients with IDs in the given
        list

        :param client_ids: list of client IDs
        :type client_ids: list
        """

        for i in client_ids:
            try:
                cls.all_clients_and_ids[i].unregister()
            except Exception as e:
                rhbplog.loginfo(msg="Error in unregistering of a delegationmanager at client with ID "+str(i)+": "+e.message)
                # not essential for running, so continue

    @classmethod
    def get_client(cls, client_id):
        """
        Returns the client with the given ID

        :param client_id: ID of the client
        :type client_id: int
        :return: matching instance of client
        :rtype: DelegationClient
        """

        return cls.all_clients_and_ids[client_id]

    def __init__(self):
        """
        Constructor of the client
        """

        self._delegation_manager = None
        self._active_manager = False
        self._active_delegations = []   # list of IDs
        DelegationClient.instance_counter += 1
        self._client_id = DelegationClient.instance_counter
        DelegationClient.all_clients_and_ids[self._client_id] = self

    def register(self, delegation_manager):
        """
        Registers a delegation manager at this client if there was none
        registered till now

        :param delegation_manager: DelegationManager from task_decomposition
                module
        :type delegation_manager: DelegationManager
        """

        if self._active_manager:
            rhbplog.logwarn("Attempt to log a new delegation_manager with the name \"" + str(delegation_manager.get_name())
                            + "\" while one with the name \"" + str(self._delegation_manager.get_name()) + "\" is already registered.\nNew DelegationManager will be ignored.")
            # will still use the old registered one
            return

        self._delegation_manager = delegation_manager
        self._active_manager = True
        delegation_manager.add_interface(interface_id=self._client_id)

    def unregister(self):
        """
        Unregisters currently used DelegationManager from this client
        """

        self._delegation_manager.remove_interface(interface_id=self._client_id)
        self._active_manager = False
        self._delegation_manager = None

    def activate(self):

        if self._active_manager:
            self._delegation_manager.activate_interface(interface_id=self._client_id)

    def deactivate(self):

        if self._active_manager:
            self._delegation_manager.deactivate_interface(interface_id=self._client_id)

    def check_if_registered(self):
        """
        Checks whether there is currently a registered delegation_manager or not

        :return: whether there is currently a registered delegation_manager or
                not
        :rtype: bool
        """

        return self._active_manager

    def do_step(self, current_step):
        """
        If a delegation manager is registered, it will make a step, if it has
        not done this step already
        """

        if self._active_manager:
            self._delegation_manager.do_step(current_step=current_step)

    def delegate(self, name, conditions, satisfaction_threshold):
        """
        Tries to delegate a goal with given parameters

        :param name: name of the goal
        :type name: str
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

        condition_string = "\n\t".join([str(x) for x in conditions])
        rhbplog.loginfo("New delegation attempt with the conditions:\n\t" + condition_string + "\n\t and the satisfaction threshold of " + str(satisfaction_threshold))

        new_goal_wrapper = RHBPGoalWrapper(name=name, conditions=conditions, satisfaction_threshold=satisfaction_threshold)

        delegation_id = self._delegation_manager.delegate(goal_wrapper=new_goal_wrapper)

        self._active_delegations.append(delegation_id)
        return delegation_id

    def terminate_delegation(self, delegation_id):
        """
        Terminates the delegation with a given ID

        :param delegation_id: ID of the delegation
        :type delegation_id: int
        """

        if self._active_manager and self._active_delegations.__contains__(delegation_id):
            self._delegation_manager.terminate(auction_id=delegation_id)
            self._active_delegations.remove(delegation_id)

    def terminate_all_delegations(self):
        """
        Terminates all current delegations of this client
        """

        for delegation_id in self._active_delegations:
            self.terminate_delegation(delegation_id=delegation_id)


class ManagerDelegationClient(DelegationClient):
    """
    Version of the DelegationClient used for Managers that handle goals and
    cost evaluation.
    """

    def __init__(self, manager):
        """
        Constructor for the client

        :param manager: a Manager from RHBP
        :type manager: Manager
        """

        super(ManagerDelegationClient, self).__init__()
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

        super(ManagerDelegationClient, self).register(delegation_manager=delegation_manager)

        if add_own_cost_evaluator:
            delegation_manager.set_cost_function_evaluator(cost_function_evaluator=self.get_new_cost_evaluator(), manager_name=self.__behaviour_manager._prefix, interface_id=self._client_id)

    def notify_goal_removal(self, goal_name):
        """
        Notifies the delegation_manager, if present, that a goal was removed

        :param goal_name: name of the removed goal
        :type goal_name: str
        """

        if self._active_manager:
            self._delegation_manager.end_task(goal_name=goal_name)

    def get_new_cost_evaluator(self):
        """
        Constructs a new cost_evaluator and returns it

        :return: a cost_evaluator using the managers planning functions
        :rtype: PDDLCostEvaluator
        """

        new_cost_evaluator = PDDLCostEvaluator(planning_function=self.__behaviour_manager.plan_with_additional_goal)

        return new_cost_evaluator
