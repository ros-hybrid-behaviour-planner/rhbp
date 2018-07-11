
from delegation_components.delegation_clients import DelegationClientBase

from decomposition_components.goal_wrapper import RHBPGoalWrapper
from decomposition_components.cost_computing import PDDLCostEvaluator


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

        new_goal_wrapper = self.build_goal_wrapper(conditions=conditions, goal_name=goal_name, satisfaction_threshold=satisfaction_threshold)

        delegation_id = self.delegate_goal_wrapper(goal_wrapper=new_goal_wrapper)
        self.logger.loginfo("Delegation has local ID " + str(delegation_id))

        return delegation_id

    def build_goal_wrapper(self, conditions, goal_name, satisfaction_threshold=1.0):
        """
        Builds a GoalWrapper from given parameters

        :param conditions: list of conditions
        :type conditions: list
        :param goal_name: name of the goal
        :type goal_name: str
        :param satisfaction_threshold: satisfaction threshold of the goal
        :type satisfaction_threshold: float
        :return: the built GoalWrapper
        :rtype: RHBPGoalWrapper
        """
        if conditions is None:
            self.logger.logwarn("Trying to delegate a goal without conditions")
            conditions = []

        condition_string = "\n\t".join([str(x) for x in conditions])

        self.logger.loginfo("New delegation attempt with the conditions:\n\t" + condition_string + "\n\t and the satisfaction threshold of " + str(satisfaction_threshold))
        new_goal_wrapper = RHBPGoalWrapper(name=goal_name, conditions=conditions, satisfaction_threshold=satisfaction_threshold)
        return new_goal_wrapper

    def start_work(self, delegation_id):
        # This should never happen
        self.logger.logerr(msg="Unexpected need to start work for the delegation "+str(delegation_id)+" in the DelegationClient with the ID "+str(self.id))


class RHBPDelegableClient(RHBPDelegationClient):
    """
    Version of the RHBPDelegationClient used for units that can do the work
    themselves if needed (the DelegableBehaviour)
    """

    def __init__(self):
        """
        Constructor
        """

        super(RHBPDelegableClient, self).__init__()
        self._work_function_dictionary = {}

    def delegate(self, goal_name, conditions=None, satisfaction_threshold=1.0, own_cost=-1, start_work_function=None):
        """
        Tries to delegate a goal with given parameters, gives a proposal
        themselves if given a positive own cost

        :param goal_name: name of the goal
        :type goal_name: str
        :param conditions: a list of conditions
        :type conditions: list
        :param satisfaction_threshold: the satisfaction threshold of the goal
        :type satisfaction_threshold: float
        :param own_cost: Cost to do the task myself, set to -1 if it cant be
                performed
        :type own_cost: float
        :param start_work_function: function pointer on function that needs to
                be invoked should i need to perform the work myself, setting
                this is important should the own_cost be set to 0 or higher
        :type start_work_function: pointer on a function
        :return: ID of the delegation
        :rtype: int
        :raises RuntimeError: if no DelegationManager is registered
        :raises RuntimeError: if own_cost>=0 and no start_work_function is given
        """

        if not self._active_manager:
            raise RuntimeError("Delegation without a registered DelegationManager")

        if own_cost >= 0 and start_work_function is None:
            raise RuntimeError("Delegation with set own cost but no given work_function")

        new_goal_wrapper = self.build_goal_wrapper(conditions=conditions, goal_name=goal_name, satisfaction_threshold=satisfaction_threshold)

        delegation_id = self.delegate_goal_wrapper(goal_wrapper=new_goal_wrapper, own_cost=own_cost)
        self.logger.loginfo("Delegation has local ID " + str(delegation_id))

        if own_cost >= 0:
            self._add_work_function(delegation_id=delegation_id, start_work_function=start_work_function)

        return delegation_id

    def _add_work_function(self, delegation_id, start_work_function):
        """
        Adds a start_work_function for the delegation with the given ID

        :param delegation_id: ID of the delegation
        :type delegation_id: int
        :param start_work_function: pointer on the start_work_function
        :type start_work_function: pointer on a function
        """

        self._work_function_dictionary[delegation_id] = start_work_function

    def start_work(self, delegation_id):
        """
        Signal the owner of this delegation that he needs to do the work
        himself, this will invoke the function given at the start of the auction

        This will be done automatically, dont call this function yourself

        :param delegation_id: ID of the delegation
        :type delegation_id: int
        :raises KeyError: if there is no function given for this delegation
        """

        self.logger.loginfo("Won the auction for the delegation "+str(delegation_id)+" myself, starting to work myself")
        try:
            func = self._work_function_dictionary[delegation_id]
        except KeyError:
            self.logger.logerr("No start_work_function given for delegation "+str(delegation_id)+": No work will be done here!")
            raise

        # start the working then return
        func()
        return

    def terminate_delegation(self, delegation_id):
        """
        Terminates the delegation with the given ID

        :param delegation_id: ID of the delegation
        :type delegation_id: int
        """

        super(RHBPDelegableClient, self).terminate_delegation(delegation_id=delegation_id)

        if self._work_function_dictionary.keys().__contains__(delegation_id):
            del self._work_function_dictionary[delegation_id]


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
