
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.delegation_interface import DelegationInterfaceBase


class DelegationBehaviour(BehaviourBase):

    def __init__(self, **kwargs):
        super(DelegationBehaviour, self).__init__(**kwargs)
        self._delegation_interface = DelegationInterfaceBase()
        self._correlation_sensors = {}

    def register_delegation_manager(self, delegation_manager):
        self._delegation_interface.register(delegation_manager=delegation_manager, use_this_for_steps=False)    # TODO think about

    def start(self):
        # TODO create conditions and delegate
        pass

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


