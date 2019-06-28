"""
DelegationGoal and the GoalWrapper used by the DelegationGoal

@author: Mengers, Hrabia
"""

from decomposition_components.delegation_clients import RHBPDelegationClient
from decomposition_components.goal_wrapper import RHBPGoalWrapper, DecompositionGoal
from delegation_components.delegation_manager import DelegationManager
from behaviour_components.goals import rhbplog


class DelegationGoal(DecompositionGoal):
    """
    Goal for the RHBP that will determine the best suited agent in the system
    and registers itself there

    Use start_auction and finish_auction to start/finish auction for best
    suited agent
    """

    def __init__(self, name, permanent=False, conditions=None, priority=0, satisfaction_threshold=1.0, enabled=True):
        """
        Constructor

        :param name: name of the goal
        :type name: str
        :param permanent: whether this goal is permanent
        :type permanent: bool
        :param conditions: conditions of this goal
        :type conditions: list
        :param priority: priority of this goal
        :type priority: int
        :param satisfaction_threshold: satisfaction_threshold of this goal
        :type satisfaction_threshold: float
        :param enabled: whether this goal is activated or not at beginning
        :type enabled: bool
        """

        self._contractor_found = False
        super(DelegationGoal, self).__init__(name=name, permanent=permanent, conditions=conditions,
                                             planner_prefix="TBD", priority=priority,
                                             satisfaction_threshold=satisfaction_threshold, enabled=enabled)
        self._client = RHBPDelegationClient()
        self._goal_wrapper = None
        self._delegation_id = -1
        self._auction_running = False

    def __del__(self):
        """
        Destructor
        """

        self._contractor_found = False
        if self._goal_wrapper is not None:
            self._goal_wrapper.__del__()
        self._client.__del__()
        super(DelegationGoal, self).__del__()

    def _init_services(self):
        """
        Initializes services, if a contractor has been found
        """

        if not self._contractor_found:
            return

        super(DelegationGoal, self)._init_services()

    def register(self):
        """
        Registers at the manager, if the right ine has been determined yet
        """

        if not self._contractor_found:
            return

        super(DelegationGoal, self).register()

    # ------ Additional Interface of the DelegationGoal ------

    def start_auction(self):
        """
        Starts the process of determining the right agent in the system for
        this goal

        Wait for some time depending on system specs before finishing auction
        """
        self.updateComputation(manager_step=None) # TODO actually this is not needed we are not transfering the state information
        self._goal_wrapper = RunningGoalWrapper(goal=self, conditions=self._conditions)
        self._delegation_id = self._client.delegate_goal_wrapper(goal_wrapper=self._goal_wrapper, known_depth=0)
        self._auction_running = True

    def finish_auction(self):
        """
        Finishes the process of determining the right agent in the system for
        this goal

        This can be used in a While-Loop, but use rospy.wait() for some time
        between the finish_auction() calls (give the messages enough time,
        depending on system specs)

        :return: Whether the process was successful or not
        :rtype: bool
        """

        if not self._auction_running:
            rhbplog.logwarn("Trying to finish an auction that has not started yet for DelegationGoal"+self.name)
            return False

        for i in range(DelegationManager.DEFAULT_AUCTION_STEPS):  # TODO make this configurable, too!
            self._client.do_step()

        if not self._contractor_found:
            rhbplog.logwarn("Auction was not finished successful (see DelegationManager logging)!" +
                            " You can try again to finish the auction after some additional time")
        else:
            self._auction_running = False
            rhbplog.loginfo("Contractor has been found for this DelegationGoal: " + self._planner_prefix)

        return self._contractor_found

    def start_with_contractor(self, planner_prefix):
        """
        Starts services and registers with the given Manager

        Can be used to ignore any auctions and simply use the given Manager

        :param planner_prefix: prefix of the Manager, that will have this goal
                now
        :type planner_prefix: str
        """

        self._planner_prefix = planner_prefix
        self._contractor_found = True
        self._init_services()
        self.register()

    def terminate_goal(self):
        """
        Unregisters this goal, but checks if it is registered before
        """

        if hasattr(self, '_registered') and self._registered:
            self.unregister(terminate_services=True)

    # ------ Properties ------

    @property
    def delegation_id(self):
        """
        The Delegation ID inside the DelegationModule for this Goal at this Node

        :return: the delegation ID
        :rtype: int
        """

        return self._delegation_id

    @property
    def auction_running(self):
        """
        Whether or not the auction for this goal is running

        :return: whether or not the auction is running
        :rtype: bool
        """

        return self._auction_running

    @property
    def contractor_found(self):
        """
        Whether or not a contractor has been found for this goal

        :return: whether or not a contractor has been found
        :rtype: bool
        """

        return self._contractor_found

    @property
    def number_of_proposals(self):

        return self._client.get_number_of_proposals(delegation_id=self._delegation_id)

    @property
    def contractor_prefix(self):
        """
        gets the planner prefix of the finally found contractor
        :return: planner prefix str
        """
        return self._planner_prefix


class RunningGoalWrapper(RHBPGoalWrapper):
    """
    Wrapper for a already running RHBP goal (GoalBase), that implements
    start_with_contractor and terminate_goal

    Goal should not be registered at the time of the construction of this
    wrapper
    """

    def __init__(self, goal, conditions):
        """
        Constructor

        :param goal: unregistered RHBP goal that will be wrapped by this
        :type goal: GoalBase
        :param conditions: list of conditions of the goal (should be exactly the
                same as the conditions of the goal)
        :type conditions: list
        """

        super(RunningGoalWrapper, self).__init__(name=goal.name, conditions=conditions,
                                                 satisfaction_threshold=goal.satisfaction_threshold)
        self._goal = goal

    def __del__(self):
        """
        Destructor
        """

        del self._conditions

    def send_goal(self, name=""):
        """
        Transfers the planner prefix of a Manager to this goal, so that it will
        register there

        :param name: planner_prefix of a Manager in the system
        :type name: str
        """

        self._goal.start_with_contractor(planner_prefix=name)
        self._created_goal = True

    def terminate_goal(self):
        """
        Makes sure the goal unregisters
        """

        self._created_goal = False
        self._goal.terminate_goal()
