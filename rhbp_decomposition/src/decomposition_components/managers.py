
import behaviour_components.managers as core
from decomposition_components.manager_client import RHBPManagerDelegationClient


class Manager(core.Manager):
    """
    This is the Manager that acts like the Manager from rhbp_core, but includes
    some additional functionality needed for the decomposition and delegation
    of tasks

    For additional documentation see the rhbp_core Manager
    """

    # noinspection PyPep8Naming
    def __init__(self, enabled=True, use_only_running_behaviors_for_interRuptible=core.Manager.USE_ONLY_RUNNING_BEHAVIOURS_FOR_INTERRUPTIBLE_DEFAULT_VALUE, **kwargs):
        """
        Constructor

        For additional documentation see the rhbp_core Manager
        """

        super(Manager, self).__init__(enabled=enabled, use_only_running_behaviors_for_interRuptible=use_only_running_behaviors_for_interRuptible, **kwargs)

        self._delegation_client = RHBPManagerDelegationClient(manager=self)
        self._participating_in_auctions = False

    def step(self, force=False, guarantee_decision=False):
        """
        Like the core Manager.step() with additional stepping of the delegation
        unit

        :param force: set to True to force the stepping regardless of pause
                and activation states
        :type force: bool
        """

        super(Manager, self).step(force=force, guarantee_decision=guarantee_decision)

        # Let the DelegationManager do a step
        self._delegation_client.do_step()  # TODO think about the position of this (step_lock y/n)

    def remove_goal(self, goal_name):
        """
        Like the core Manager.remove_goal() with additional notification of
        the delegation unit

        :param goal_name: name of the goal, that should be removed
        :type goal_name: str
        """

        super(Manager, self).remove_goal(goal_name=goal_name)

        # notify the delegation unit that a goal is removed
        self._delegation_client.notify_goal_removal(goal_name=goal_name)

    def participate_in_auctions(self):
        self._delegation_client.make_cost_computable()
        self._participating_in_auctions = True

    def stop_participating_in_auctions(self):
        self._delegation_client.remove_cost_computable()
        self._participating_in_auctions = False

    @property
    def participating_in_auctions(self):
        return self._participating_in_auctions

    @participating_in_auctions.setter
    def participating_in_auctions(self, value):
        if self._participating_in_auctions and not value:
            self.stop_participating_in_auctions()

        if not self._participating_in_auctions and value:
            self.participate_in_auctions()

    @property
    def delegation_client(self):
        """
        :return: The delegation client used by this manager
        :rtype: RHBPManagerDelegationClient
        """

        return self._delegation_client
