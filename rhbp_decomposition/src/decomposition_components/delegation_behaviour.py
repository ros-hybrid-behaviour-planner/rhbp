"""
DelegationBehaviour and DelegableBehaviour

@author: Mengers
"""

from behaviour_components.behaviours import BehaviourBase, rhbplog
from decomposition_components.delegation_clients import RHBPDelegationClient, RHBPDelegableClient
from behaviour_components.conditions import create_condition_from_effect
from delegation_components.delegation_errors import DelegationError
from abc import abstractmethod


class DelegationBehaviour(BehaviourBase):
    """
    A behaviour that will delegate a task containing all effects, that this
    behaviour has, after it was started.

    Needs NO implementation of the functions start, stop, do_step, but needs
    Effects added via the function add_effect_for_delegation(), if they shall be
    part of the delegated task.
    """

    TYPE_STRING = "Delegation"

    # noinspection PyPep8Naming
    def __init__(self, name, planner_prefix, satisfaction_threshold=1.0,
                 delegation_depth_prefix=None, **kwargs):
        """
        Constructor

        :param name: name of the Behaviour
        :type name: str
        :param planner_prefix: prefix of the manager, this behaviour should
                register at
        :type planner_prefix: str
        :param satisfaction_threshold: the satisfaction_threshold used for the
                goal, that will be created by this behaviour
        :type satisfaction_threshold: float
        :param delegation_depth_prefix: (optional) if this is part of a
                NetworkBehaviour or any other arrangement, so that for depth
                checking a specific Manager should be considered that is not the
                direct Manager of this Behaviour, give the prefix of that
                Manager here, else leave it empty
        :type delegation_depth_prefix: str
        :param kwargs: arguments passed to the Constructor of the BaseClass,
                for documentation see the Constructor of BehaviourBase
        """

        if delegation_depth_prefix is None:
            checking_prefix = planner_prefix
        else:
            checking_prefix = delegation_depth_prefix

        super(DelegationBehaviour, self).__init__(name=name,
                                                  planner_prefix=planner_prefix,
                                                  requires_execution_steps=True,
                                                  **kwargs)
        self._correlation_sensors = {}
        self._satisfaction_threshold = satisfaction_threshold
        self._create_delegation_client(checking_prefix=checking_prefix)
        self._delegation_conditions = []
        self._attempt_unsuccessful = False

    def __del__(self):
        """
        Destructor
        """

        super(DelegationBehaviour, self).__del__()
        self._delegation_client.__del__()
        del self._correlation_sensors[:]

    # ------ Internal Methods ------

    def _create_delegation_client(self, checking_prefix):
        """
        Creates an instance of the delegation client, will be done by the
        Constructor
        """

        self._delegation_client = RHBPDelegationClient(checking_prefix=checking_prefix)

    def _delegate(self):
        """
        Starts the delegation with all current effects as conditions for the goal
        """

        conditions = self._get_conditions_for_delegation()
        try:
            self._delegation_client.delegate(goal_name=self.name + "Goal", conditions=conditions,
                                             satisfaction_threshold=self._satisfaction_threshold,
                                             success_function=self._finish_after_success)
            self._attempt_unsuccessful = False
        except DelegationError:
            rhbplog.logwarn("Attempted Delegation was not successful. Will retry next step!")
            self._attempt_unsuccessful = True

    def _finish_after_success(self):
        """
        Stops own execution after a successful delegation
        """

        rhbplog.loginfo("DelegationBehaviour " + self.name
                        + " was successful with the delegation, stopping own execution")
        self.finish(stop_running=True)

    def _get_conditions_for_delegation(self):
        """
        Builds a list of conditions for all known effects, that have a known
        corresponding sensor to them

        :return: list of conditions
        :rtype: list
        """

        conditions = []

        for effect in self.correlations:
            # only effects with matching sensors become conditions
            if self._correlation_sensors.keys().__contains__(effect):
                sensor = self._correlation_sensors[effect]
                condition = create_condition_from_effect(effect=effect,
                                                         sensor=sensor)
                conditions.append(condition)

        conditions.extend(self._delegation_conditions)

        return conditions

    # ------ Start, Stop, Do_Step ------

    def start(self):
        """
        Does start a delegation with all known effects, that have a known
        corresponding sensor to them, as conditions for the goal
        """

        if not self._delegation_client.check_if_registered():
            rhbplog.logerr("DelegationBehaviour %s started without" 
                           "a registered DelegationManager. Will do nothing!",
                           self.name)
            return

        self._delegate()

    def stop(self):
        """
        Terminates any active delegation
        """

        self._delegation_client.terminate_all_delegations()

    def do_step(self):
        """
        Makes sure the auction will step, if an auction is active
        """

        if self._attempt_unsuccessful:
            self._delegate()
            return

        self._delegation_client.do_step()

    # ------ Adding Effects and delegation conditions ------

    def add_effect(self, effect):
        """
        Adds an effect to this behaviour that will be considered by the planner,
        but will not actually be part of the delegation!

        :param effect: effect to be added
        :type effect: Effect
        """

        rhbplog.logdebug(msg="Effect added via add_effect to a DelegationBehaviour" 
                             "wont be actually part of the delegated goal")

        super(DelegationBehaviour, self).add_effect(effect=effect)

    def add_effect_for_delegation(self, effect, sensor):
        """
        Adds an effect to this behaviour and uses the corresponding sensor to
        build conditions for a later delegated goal

        :param effect: effect to be added
        :type effect: Effect
        :param sensor: effected sensor
        :type sensor: Sensor
        """

        super(DelegationBehaviour, self).add_effect(effect=effect)
        self._correlation_sensors[effect] = sensor

    def add_condition_for_delegation(self, condition):
        """
        Adds this condition to the goal, that will be delegated

        :param condition: a condition
        """

        rhbplog.logdebug(msg="There has to be a suiting Effect added to this DelegationBehaviour," +
                             "if this condition for delegation should be considered by the planner as effect")

        self._delegation_conditions.append(condition)

    @property
    def delegation_client(self):
        """
        The used DelegationClient

        :return: the DelegationClient
        :rtype: DelegationClient
        """

        return self._delegation_client


class DelegableBehaviour(DelegationBehaviour):
    """
    Like the DelegationBehaviour, but there is the possibility to do the work
    here locally. The decision whether it will be done locally or will be
    delegated is done by the DelegationManager of the delegation_module
    based on the given own_cost in comparision to the proposals made by possible
    contractors.

    Needs the implementation of start_work, do_step_work and stop_work like
    BehaviourBase needs start, do_step and stop. Needs effects added via the
    function add_effect_for_delegation() if they shall be considered for the
    delegation, just like the DelegationBehaviour.
    """

    # noinspection PyPep8Naming
    def __init__(self, name, planner_prefix, work_cost,
                 satisfaction_threshold=1.0, delegation_depth_prefix=None,
                 **kwargs):
        """
        Constructor

        :param name: name of the Behaviour
        :type name: str
        :param planner_prefix: prefix of the manager, this behaviour should
                register at
        :type planner_prefix: str
        :param work_cost: cost of doing the work myself, just needs to be right
                relatively to the cost of other units
        :type work_cost: float
        :param satisfaction_threshold: the satisfaction_threshold used for the
                goal, that will be created by this behaviour
        :type satisfaction_threshold: float
        :param delegation_depth_prefix: (optional) if this is part of a
                NetworkBehaviour or any other arrangement, so that for depth
                checking a specific Manager should be considered that is not the
                direct Manager of this Behaviour, give the prefix of that
                Manager here, else leave it empty
        :type delegation_depth_prefix: str
        :param kwargs: arguments passed to the Constructor of the BaseClass,
                for documentation see the Constructor of BehaviourBase
        """

        super(DelegableBehaviour, self).__init__(name=name, planner_prefix=planner_prefix,
                                                 satisfaction_threshold=satisfaction_threshold,
                                                 delegation_depth_prefix=delegation_depth_prefix,
                                                 **kwargs)
        self._own_cost = work_cost
        self._currently_doing_work_myself = False

    # ------ Internal Methods ------

    def _create_delegation_client(self, checking_prefix):
        """
        Creates an instance of the delegation client, will be done by the
        Constructor
        """

        self._delegation_client = RHBPDelegableClient(checking_prefix=checking_prefix)

    def _quick_start_work(self):
        """
        Starts up to do the work locally, used to start the work from outside
        this class

        Do not invoke this manually
        """

        self._currently_doing_work_myself = True
        self.start_work()

    # ------ Start, Stop, Do_Step ------

    def start(self):
        """
        Starts an auction for this task, if possible, else it just starts to do
        the work
        """

        if self._delegation_client.check_if_registered():
            conditions = self._get_conditions_for_delegation()
            try:
                self._delegation_client.delegate(goal_name=self.name + "Goal",
                                                 conditions=conditions,
                                                 satisfaction_threshold=self._satisfaction_threshold,
                                                 own_cost=self._own_cost,
                                                 start_work_function=self._quick_start_work,
                                                 success_function=self._finish_after_success)
            except Exception as e:
                rhbplog.logwarn("Attempted Delegation was not successful (error-msg: "
                                + str(e.message) + ")\n\t-->Will do work myself!")
                self._quick_start_work()
                return

            self._currently_doing_work_myself = False

        else:
            self._quick_start_work()

    def stop(self):
        """
        Stops doing the work or terminates the task, depending on the state
        """

        if self._currently_doing_work_myself:
            self.stop_work()
            self._currently_doing_work_myself = False

        else:
            super(DelegableBehaviour, self).stop()

    def do_step(self):
        """
        Makes a step for the auction, if running, or does a step of work,
        depending on the state
        """

        if self._currently_doing_work_myself:
            self.do_step_work()

        else:
            super(DelegableBehaviour, self).do_step()

    # ------ Abstract Functions if Work has to be done locally ------

    @abstractmethod
    def start_work(self):
        """
        All starting up of work for this behaviour, if the work has to be done
        locally. Just like BehaviourBase.start()

        Needs to be overridden!

        :return: None
        """

        raise NotImplementedError()

    @abstractmethod
    def stop_work(self):
        """
        All stopping of work for this behaviour, if the work was done
        locally. Just like BehaviourBase.stop()

        Needs to be overridden!

        :return: None
        """

        raise NotImplementedError()

    @abstractmethod
    def do_step_work(self):
        """
        Steps the work of this behaviour, if the work has to be done locally.
        Just like BehaviourBase.do_step()

        If there are no steps needed for doing the work, implement this as pass.

        Needs to be overridden!

        :return: None
        """

        raise NotImplementedError()

    # ------ Properties ------

    @property
    def currently_doing_work_locally(self):
        """
        Whether the work is done locally or not

        :return: Whether the work is done locally or not
        :rtype: bool
        """

        return self._currently_doing_work_myself
