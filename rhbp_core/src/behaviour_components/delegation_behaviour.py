
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.delegation_interface import DelegationInterfaceBase


class DelegationBehaviour(BehaviourBase):

    def __init__(self, name, requires_execution_steps=False, satisfaction_threshold=1.0, **kwargs):
        super(DelegationBehaviour, self).__init__(name=name, requires_execution_steps=requires_execution_steps, **kwargs)
        self._delegation_interface = DelegationInterfaceBase()
        self._correlation_sensors = {}
        self._satisfaction_threshold = satisfaction_threshold

    def register_delegation_manager(self, delegation_manager):
        self._delegation_interface.register(delegation_manager=delegation_manager, use_this_for_steps=False)    # TODO think about

    def start(self):
        conditions = []
        for effect in self.correlations:
            sensor = self._correlation_sensors[effect]  # TODO catch exceptions?
            condition = effect.create_condition_from_effect(sensor=sensor)
            conditions.append(condition)

        self._delegation_interface.delegate(name=self.name+"Goal", conditions=conditions, satisfaction_threshold=self._satisfaction_threshold)

    def do_step(self):
        self._delegation_interface.do_step()    # TODO make sure only one step will be done

    def stop(self):
        # TODO end all auctions and stuff the delegation_manager did for us
        pass

    def add_effect(self, effect, sensor=None):

        if sensor is None:
            raise RuntimeError(msg="DelegationBehaviours need to know the sensor to an effect")

        super(DelegationBehaviour, self).add_effect(effect=effect)
        self._correlation_sensors[effect] = sensor


