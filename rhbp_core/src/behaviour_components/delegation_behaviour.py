
from behaviour_components.behaviours import BehaviourBase, rhbplog
from delegation_components.delegation_clients import DelegationClient
from behaviour_components.conditions import create_condition_from_effect


class DelegationBehaviour(BehaviourBase):

    def __init__(self, name, satisfaction_threshold=1.0, **kwargs):

        super(DelegationBehaviour, self).__init__(name=name, requires_execution_steps=True, **kwargs)
        self._delegation_interface = DelegationClient()
        self._correlation_sensors = {}
        self._satisfaction_threshold = satisfaction_threshold

    def __del__(self):

        super(DelegationBehaviour, self).__del__()
        self._delegation_interface.__del__()
        del self._correlation_sensors[:]

    def register_delegation_manager(self, delegation_manager):
        self._delegation_interface.register(delegation_manager=delegation_manager)

    def start(self):
        """
        Does start a delegation with all known effects, that have a known
        corresponding sensor to them, as conditions for the goal
        """

        if not self._delegation_interface.check_if_registered():
            rhbplog.logerr("DelegationBehaviour %s started without a registered DelegationManager. Will do nothing!", self.name)
            return

        conditions = self._get_conditions_for_delegation()

        self._delegation_interface.delegate(name=self.name+"Goal", conditions=conditions, satisfaction_threshold=self._satisfaction_threshold)

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
                condition = create_condition_from_effect(effect=effect, sensor=sensor)
                conditions.append(condition)

        return conditions

    def do_current_step(self, current_step):
        self._delegation_interface.do_step(current_step=current_step)

    def stop(self):
        self._delegation_interface.terminate_all_delegations()

    def add_effect(self, effect):
        """
        Adds an effect to this behaviour that will be considered by the planner,
        but will not actually be part of the delegation!

        :param effect: effect to be added
        """

        rhbplog.logwarn(msg="Effect added via add_effect to a DelegationBehaviour wont be actually part of the delegated goal")

        super(DelegationBehaviour, self).add_effect(effect=effect)

    def add_effect_for_delegation(self, effect, sensor):
        """
        Adds an effect to this behaviour and uses the corresponding sensor to
        build conditions for a later delegated goal

        :param effect: effect to be added
        :param sensor: effected sensor
        """

        super(DelegationBehaviour, self).add_effect(effect=effect)
        self._correlation_sensors[effect] = sensor


class DelegableBehaviour(DelegationBehaviour):

    def __init__(self, name, work_cost, satisfaction_threshold=1.0, **kwargs):
        super(DelegableBehaviour, self).__init__(name=name, satisfaction_threshold=satisfaction_threshold, **kwargs)
        self._own_cost = work_cost
        self._currently_doing_work_myself = False

    def start(self):
        if self._delegation_interface.check_if_registered():

            conditions = self._get_conditions_for_delegation()
            # TODO own work cost as parameter, delegate

            self._currently_doing_work_myself = False
        else:

            self.start_work()
            self._currently_doing_work_myself = True

    def stop(self):
        if self._currently_doing_work_myself:
            self.stop_work()
            self._currently_doing_work_myself = False

        else:
            super(DelegableBehaviour, self).stop()

    def do_step(self):
        if self._currently_doing_work_myself:
            self.do_step_work()

        else:
            super(DelegableBehaviour, self).do_step()

    def start_work(self):
        raise NotImplementedError()

    def stop_work(self):
        raise NotImplementedError()

    def do_step_work(self):
        raise NotImplementedError()
