"""
Manager for the RHBP that has to be used by Nodes that want to include the
rhbp_decomposition capabilities

@author: Mengers, Hrabia
"""

import rospy

import behaviour_components.managers as core
from decomposition_components.manager_client import RHBPManagerDelegationClient

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.delegation')


class Manager(core.Manager):
    """
    This is the Manager that acts like the Manager from rhbp_core, but includes
    some additional functionality needed for the decomposition and delegation
    of tasks

    For additional documentation see the rhbp_core Manager
    """

    # noinspection PyPep8Naming
    def __init__(self, enabled=True,
                 use_only_running_behaviors_for_interRuptible=core.Manager.USE_ONLY_RUNNING_BEHAVIOURS_FOR_INTERRUPTIBLE_DEFAULT_VALUE,
                 **kwargs):
        """
        Constructor

        For additional documentation see the rhbp_core Manager
        """

        super(Manager, self).__init__(enabled=enabled,
                                      use_only_running_behaviors_for_interRuptible=use_only_running_behaviors_for_interRuptible,
                                      **kwargs)

        self._delegation_client = RHBPManagerDelegationClient(manager=self)

        self._participating_in_auctions = kwargs['participating_in_auctions'] if 'participating_in_auctions' in kwargs else \
            rospy.get_param(self._param_prefix + "/participating_in_auctions", False)
        if self._participating_in_auctions:
            self.participate_in_auctions()
        rhbplog.loginfo("Manager %s is participating in auctions: %s", self._prefix, self._participating_in_auctions)

    def step(self, force=False, guarantee_decision=False):
        """
        Like the core Manager.step() with additional stepping of the delegation
        unit

        :param force: set to True to force the stepping regardless of pause
                and activation states
        :type force: bool
        :param guarantee_decision: Repeat activation algorithm threshold
                adjustments until at least one behaviour can be activated
                and enough executable behaviours are available
        :type guarantee_decision: bool
        """

        super(Manager, self).step(force=force, guarantee_decision=guarantee_decision)

        # Let the DelegationManager do a step and update the pursued goals
        self._delegation_client.do_step()
        self._delegation_client.update_actively_pursued_goals(current_active_goals=self._currently_pursued_goals)

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
        """
        Enables this Manager to participate in auctions
        """

        self._delegation_client.make_cost_computable()
        self._participating_in_auctions = True

    def stop_participating_in_auctions(self):
        """
        Disables this Manager to participate in auctions
        """

        self._delegation_client.remove_cost_computable()
        self._participating_in_auctions = False

    @property
    def participating_in_auctions(self):
        """
        Whether this Manager can participate in auctions or not

        :return: Whether this Manager can participate in auctions or not
        :rtype: bool
        """

        return self._participating_in_auctions

    @participating_in_auctions.setter
    def participating_in_auctions(self, value):
        """
        Whether this Manager should participate in auctions or not

        :param value: Whether this Manager should participate in auctions or not
        :type value: bool
        """

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
