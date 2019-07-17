"""
Delegation Clients for components, that do not participate in auctions

@author: Mengers, Hrabia
"""

from delegation_components.delegation_clients import DelegationClientBase
from delegation_components.delegation_manager import DelegationManager
from decomposition_components.goal_wrapper import RHBPGoalWrapper
from delegation_components.delegation_errors import DelegationError
import rospy
import utils.rhbp_logging

rhbplogger = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.delegation')


class RHBPDelegationClient(DelegationClientBase):
    """
    DelegationClient for the RHBP if no tasks can be taken and no cost can be
    evaluated
    """

    # use the defined rhbplogger
    logger = rhbplogger
    # only one passive DelegationManager per Node
    used_delegation_manager = None

    def __init__(self, checking_prefix=None):
        """
        Constructor

        :param checking_prefix: prefix of a Manager, that should be consulted
                for the actual depth of the delegations via this client. Used
                for clients of Behaviours etc. that might not have the same
                DelegationManager as the responsible Manager of the component
        :type checking_prefix: str
        """

        super(RHBPDelegationClient, self).__init__()

        # create a new DelegationManager, if there is none
        if not RHBPDelegationClient.used_delegation_manager:
            self._create_delegation_manager()

        # register the DelegationManager
        super(RHBPDelegationClient, self).register(delegation_manager=RHBPDelegationClient.used_delegation_manager)
        self._active_manager = True
        self._checking_prefix = checking_prefix

        self._success_function_dict = dict()

    def __del__(self):
        """
        Destructor
        """

        super(RHBPDelegationClient, self).__del__()

    def _create_delegation_manager(self):
        """
        Creates a new instance of the DelegationManager to be used by the all
        clients
        """

        self.logger.loginfo("Creating a new DelegationManager with the name \"" + str(rospy.get_name()) + "\"")
        RHBPDelegationClient.used_delegation_manager = DelegationManager(name=rospy.get_name())

    # ------ Open Interface, interaction with delegations ------

    def delegate(self, goal_name, conditions=None, success_function=None, satisfaction_threshold=1.0):
        """
        Tries to delegate a goal with given parameters

        :param goal_name: name of the goal
        :type goal_name: str
        :param conditions: a list of conditions
        :type conditions: list
        :param success_function: function pointer to a function that should be
                executed, if the delegation is successful, can be left None, if
                not needed
        :type success_function: function pointer
        :param satisfaction_threshold: the satisfaction threshold of the goal
        :type satisfaction_threshold: float
        :return: ID of the delegation
        :rtype: int
        :raises RuntimeError: if no DelegationManager is registered
        :raises DelegationError: if Delegation was not successful
        """

        if not self._active_manager:
            raise RuntimeError("Delegation without a registered DelegationManager")

        depth = self._determine_known_depth()
        new_goal_wrapper = self._build_goal_wrapper(conditions=conditions, goal_name=goal_name, satisfaction_threshold=satisfaction_threshold)

        try:
            delegation_id = self.delegate_goal_wrapper(goal_wrapper=new_goal_wrapper, known_depth=depth)
        except DelegationError:
            self.logger.logwarn("Attempted Delegation was not successful")
            raise

        if success_function is not None:
            self._set_success_function(delegation_id=delegation_id, success_function=success_function)

        self.logger.loginfo("Delegation has local ID " + str(delegation_id))
        return delegation_id

    def get_number_of_proposals(self, delegation_id):
        """
        Determine the number of already available proposals for a delegation
        :param delegation_id: ID of the delegation
        :type delegation_id: int
        :return: number of proposals
        """
        return self._delegation_manager.get_delegation(delegation_id).number_of_proposals

    def terminate_delegation(self, delegation_id):
        """
        Terminates the delegation with the given ID

        :param delegation_id: ID of the delegation
        :type delegation_id: int
        """

        super(RHBPDelegationClient, self).terminate_delegation(delegation_id=delegation_id)
        if self._success_function_dict.keys().__contains__(delegation_id):
            del self._success_function_dict[delegation_id]

    def delegation_successful(self, delegation_id):
        """
        Signals all important components that the delegation with the given ID
        was successful, executes success_function of this delegation, if
        applicable

        :param delegation_id: ID of the delegation that was successful
        :type delegation_id: int
        """

        try:
            if self._success_function_dict.__contains__(delegation_id):
                func = self._success_function_dict[delegation_id]
                func()
            return
        except Exception as e:
            self.logger.logerr(msg="Error in success_function for delegation with ID "+str(delegation_id)
                                   + "; Error message:\""+str(e.message)+"\"; Will continue doing work.")

    # ------ helper functions for delegate ------

    def _build_goal_wrapper(self, conditions, goal_name, satisfaction_threshold=1.0):
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

    def _determine_known_depth(self):
        """
        Determines if depth can be known by this client (based on checking prefix)
        and determines depth if it is possible

        :return: depth, if checking possible, or None
        :rtype: int
        """

        if (not self._delegation_manager.depth_checking_possible) and (self._checking_prefix is not None):
            depth = self._delegation_manager.check_remote_depth(prefix=self._checking_prefix)
        else:
            depth = None
        return depth

    def _set_success_function(self, delegation_id, success_function):
        """
        Sets a success_function for the delegation with this ID

        :param delegation_id: ID of the delegation
        :type delegation_id: int
        :param success_function: function pointer to the success_function
        :type success_function: function pointer
        """

        self._success_function_dict[delegation_id] = success_function

    # ------ Functions not used by this Type of Client ------

    def start_work_for_delegation(self, delegation_id):
        """
        This should not be executed and is only a stub, used by other
        implementations
        """

        # This should never happen
        self.logger.logerr(msg="Unexpected need to start work for the delegation "+str(delegation_id)+" in the DelegationClient with the ID "+str(self.id))

    def register(self, delegation_manager):
        """
        Not needed in rhbp_decomposition!
        """

        super(RHBPDelegationClient, self).register(delegation_manager=delegation_manager)


class RHBPDelegableClient(RHBPDelegationClient):
    """
    Version of the RHBPDelegationClient used for units that can do the work
    themselves if needed (e.g. the DelegableBehaviour)
    """

    def __init__(self, checking_prefix=None):
        """
        Constructor

        :param checking_prefix: prefix of a Manager, that should be consulted
                for the actual depth of the delegations via this client. Used
                for clients of Behaviours etc. that might not have the same
                DelegationManager as the responsible Manager of the component
        :type checking_prefix: str
        """

        super(RHBPDelegableClient, self).__init__(checking_prefix=checking_prefix)
        self._work_function_dictionary = dict()

    # ------ Open Interface, interaction with delegations ------

    def delegate(self, goal_name, conditions=None, success_function=None, satisfaction_threshold=1.0, own_cost=-1, start_work_function=None):
        """
        Tries to delegate a goal with given parameters, gives a proposal
        themselves if given a positive own cost

        :param goal_name: name of the goal
        :type goal_name: str
        :param conditions: a list of conditions
        :type conditions: list
        :param success_function: function pointer to a function that should be
                executed, if the delegation is successful, can be left None, if
                not needed
        :type success_function: function pointer
        :param satisfaction_threshold: the satisfaction threshold of the goal
        :type satisfaction_threshold: float
        :param own_cost: Cost to do the task myself, set to -1 if it cant be
                performed
        :type own_cost: float
        :param start_work_function: function pointer on function that needs to
                be invoked should i need to perform the work myself, setting
                this is important should the own_cost be set to 0 or higher
        :type start_work_function: function pointer
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

        depth = self._determine_known_depth()
        new_goal_wrapper = self._build_goal_wrapper(conditions=conditions, goal_name=goal_name, satisfaction_threshold=satisfaction_threshold)

        try:
            delegation_id = self.delegate_goal_wrapper(goal_wrapper=new_goal_wrapper, own_cost=own_cost, known_depth=depth)
        except DelegationError:
            self.logger.logwarn("Attempted Delegation was not successful")
            raise

        self._add_function_pointers(delegation_id=delegation_id, own_cost=own_cost,
                                    start_work_function=start_work_function, success_function=success_function)

        self.logger.loginfo("Delegation has local ID " + str(delegation_id))
        return delegation_id

    def start_work_for_delegation(self, delegation_id):
        """
        Signal the owner of this delegation that he needs to do the work
        himself, this will invoke the function given at the start of the auction

        This will be done automatically, do not call this function yourself

        :param delegation_id: ID of the delegation
        :type delegation_id: int
        :raises KeyError: if there is no function given for this delegation
        :raises Exception: re-raises any exceptions, that are raised by the
                work function
        """

        self.logger.loginfo("Won the auction for the delegation " +
                            str(delegation_id) + " myself, starting to work myself")
        try:
            work_func = self._work_function_dictionary[delegation_id]
        except KeyError:
            self.logger.logerr("No start_work_function given for delegation " +
                               str(delegation_id) + ": No work will be done here!")
            raise

        # try to start the working then return
        try:
            work_func()
            return
        except Exception as e:
            self.logger.logerr("Error in work function of the delegation with the ID " +
                               str(delegation_id) + ":\n\t" + str(e.message))
            raise e

    def terminate_delegation(self, delegation_id):
        """
        Terminates the delegation with the given ID

        :param delegation_id: ID of the delegation
        :type delegation_id: int
        """

        super(RHBPDelegableClient, self).terminate_delegation(delegation_id=delegation_id)

        if self._work_function_dictionary.keys().__contains__(delegation_id):
            del self._work_function_dictionary[delegation_id]

    # ------ helper functions for delegate ------

    def _add_function_pointers(self, delegation_id, own_cost, start_work_function, success_function):
        """
        Adds a function pointers for success and start_work function, if
        applicable

        :param delegation_id: ID of the delegation
        :type delegation_id: int
        :param own_cost: own cost for this delegation
        :type own_cost: float
        :param start_work_function: pointer on the start_work_function
        :type start_work_function: function pointer
        :param success_function: pointer on the success_function
        :type success_function: function pointer
        """

        if success_function is not None:
            self._set_success_function(delegation_id=delegation_id, success_function=success_function)
        if own_cost >= 0:
            self._add_work_function(delegation_id=delegation_id, start_work_function=start_work_function)

    def _add_work_function(self, delegation_id, start_work_function):
        """
        Adds a start_work_function for the delegation with the given ID

        :param delegation_id: ID of the delegation
        :type delegation_id: int
        :param start_work_function: pointer on the start_work_function
        :type start_work_function: function pointer
        """

        self._work_function_dictionary[delegation_id] = start_work_function
