
from delegation_components.delegation_clients import DelegationClientBase

from decomposition_components.goal_wrapper import RHBPGoalWrapper
from delegation_components.delegation_errors import DelegationError


import utils.rhbp_logging
rhbplogger = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.delegation')


class RHBPDelegationClient(DelegationClientBase):
    """
    DelegationClient for the RHBP if no tasks can be taken and no cost be
    evaluated
    """

    logger = rhbplogger

    def __init__(self, checking_prefix):
        super(RHBPDelegationClient, self).__init__()
        self._waiting_delegations = []
        self._checking_prefix = checking_prefix

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
        :raises DelegationError: if Delegation was not successful
        """

        if not self._active_manager:
            raise RuntimeError("Delegation without a registered DelegationManager")

        if not self._delegation_manager.depth_checking_possible:
            depth = self._delegation_manager.check_remote_depth(prefix=self._checking_prefix)
        else:
            depth = None

        new_goal_wrapper = self.build_goal_wrapper(conditions=conditions, goal_name=goal_name, satisfaction_threshold=satisfaction_threshold)

        try:
            delegation_id = self.delegate_goal_wrapper(goal_wrapper=new_goal_wrapper, known_depth=depth)
        except DelegationError:
            self.logger.logwarn("Attempted Delegation was not successful")
            raise

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

    def delegation_successful(self, delegation_id):
        pass    # TODO inform behaviour that its done


class RHBPDelegableClient(RHBPDelegationClient):
    """
    Version of the RHBPDelegationClient used for units that can do the work
    themselves if needed (the DelegableBehaviour)
    """

    def __init__(self, checking_prefix):
        """
        Constructor
        """

        super(RHBPDelegableClient, self).__init__(checking_prefix=checking_prefix)
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
        :raises DelegationError: if Delegation was not successful
        """

        if not self._active_manager:
            raise RuntimeError("Delegation without a registered DelegationManager")

        if own_cost >= 0 and start_work_function is None:
            raise RuntimeError("Delegation with set own cost but no given work_function")

        if not self._delegation_manager.depth_checking_possible:
            depth = self._delegation_manager.check_remote_depth(prefix=self._checking_prefix)
        else:
            depth = None

        new_goal_wrapper = self.build_goal_wrapper(conditions=conditions, goal_name=goal_name, satisfaction_threshold=satisfaction_threshold)

        try:
            delegation_id = self.delegate_goal_wrapper(goal_wrapper=new_goal_wrapper, own_cost=own_cost, known_depth=depth)
        except DelegationError:
            self.logger.logwarn("Attempted Delegation was not successful")
            raise

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

        This will be done automatically, do not call this function yourself

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


