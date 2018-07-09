
import behaviour_components.managers as core
from decomposition_components.delegation_clients import RHBPManagerDelegationClient


class Manager(core.Manager):
    """
    This is the Manager that acts like the Manager from rhbp_core, but includes
    some additional functionalities needed for the decomposition and delegation
    of tasks

    For additional documentation see the rhbp_core Manager
    """

    def __init__(self, activated=True, use_only_running_behaviors_for_interRuptible=core.Manager.USE_ONLY_RUNNING_BEHAVIOURS_FOR_INTERRUPTIBLE_DEFAULT_VALUE, **kwargs):
        """
        Constructor

        For additional documentation see the rhbp_core Manager
        """

        super(Manager, self).__init__(activated=activated, use_only_running_behaviors_for_interRuptible=use_only_running_behaviors_for_interRuptible, **kwargs)

        # needs to know the manager for potential usage of methods
        self.__delegation_client = RHBPManagerDelegationClient(manager=self)

    def step(self, force=False):
        """
        Like the core Manager.step() with additional stepping of the delegation
        unit

        :param force: set to True to force the stepping regardless of pause
                and activation states
        :type force: bool
        """

        super(Manager, self).step(force=force)

        # Let the DelegationManager do a step
        self.__delegation_client.do_step()  # TODO think about the position of this (step_lock y/n)

    def remove_goal(self, goal_name):
        """
        Like the core Manager.remove_goal() with additional notification of
        the delegation unit

        :param goal_name: name of the goal, that should be removed
        :type goal_name: str
        """

        super(Manager, self).remove_goal(goal_name=goal_name)

        # notify the delegation unit that a goal is removed
        self.__delegation_client.notify_goal_removal(goal_name=goal_name)

    @property
    def delegation_client(self):
        """
        :return: The delegation client used by this manager
        :rtype: RHBPManagerDelegationClient
        """

        return self.__delegation_client
