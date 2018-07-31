import time

import numpy

from behaviour_components.behaviours import BehaviourBase
from behaviour_components.conditions import Condition
from behaviour_components.sensors import Sensor


class TestBaseEnvironment(object):
    # this class handels the communication with the environment and the rhbp components
    def __init__(self, env, manager, sensors):
        self.env = env
        self.manager = manager
        self.sensors = sensors
        self.frequency = 0.005
        self.count = 0
        self.last_none = True
        self.manager.__activationDecay = 1.0

    def behaviour_to_index(self, name):
        """
        
        :param name: 
        :return: 
        """
        num = 0
        for b in self.manager.behaviours:
            if b == name:
                return num
            num += 1
        return None

    def start_simulation(self):
        while (True):
            time.sleep(self.frequency)
            # print("dadadada")
            self.count += 1

            # if self.count % 100 == 0:
            # print(self.count)
            # print(self.manager.behaviours)
            self.manager.step()

            executed_behaviors = self.manager.executedBehaviours
            num_actions = len(executed_behaviors)
            # check if no action was selected. Then repeat this cycle
            if num_actions > 0:
                # ex_index = executed_behaviors[0].index
                if self.last_none:
                    self.last_none = False
                else:
                    ex_index = self.behaviour_to_index(executed_behaviors[0])
                    self.execute_step(ex_index)
            else:
                self.last_none = True

    def execute_step(self, index):
        raise NotImplementedError


class TestFrozenLakeEnv(TestBaseEnvironment):
    def __init__(self, env, manager, sensors):
        super(TestFrozenLakeEnv, self).__init__(env, manager, sensors)
        self.sensor = sensors[0]
        self.reward_sensor = sensors[1]
        self.last_state = 0

    def execute_step(self, index):
        # print("execute step",index)
        # last_state = self.sensor.value
        state, reward, isEnded, _ = self.env.step(index)

        if isEnded and reward == 0:
            reward = -1

        # print("ex1: ", index, "last state", self.last_state,
        #      "new state",state,"with reward:",reward)

        self.reward_sensor.update(reward)
        if reward == -1:
            self.reward_sensor.updates_evaluation(reward, isEnded, ignore=True)
        else:
            self.reward_sensor.updates_evaluation(reward, isEnded)
        # print("ex: ", self.index, "last state",last_state,
        #      "new state",state,"with reward:",reward)

        if isEnded:
            state = self.env.reset()
        # print("update sensor with state",state)
        self.sensor.update(state)
        self.sensor.sync()

        self.last_state = state


class TestTaxiEnv(TestBaseEnvironment):
    def __init__(self, env, manager, sensors):
        super(TestTaxiEnv, self).__init__(env, manager, sensors)
        self.reward_sensor = sensors[5]
        self.sensor_row = sensors[0]
        self.sensor_col = sensors[1]
        self.sensor_passenger = sensors[2]
        self.sensor_destination = sensors[3]
        self.state_sensor = sensors[4]
        self.environment = env
        self.last_state = 0

    def encode(self, taxirow, taxicol, passloc, destidx):
        # (5) 5, 5, 4
        i = taxirow
        i *= 5
        i += taxicol
        i *= 5
        i += passloc
        i *= 4
        i += destidx
        return i

    def decode(self, i):
        out = []
        out.append(i % 4)  # row
        i = i // 4
        out.append(i % 5)  # col
        i = i // 5
        out.append(i % 5)  # passloc
        i = i // 5
        out.append(i)  # destination
        assert 0 <= i < 5
        return reversed(out)

    def execute_step(self, index):

        try:
            # last_state = self.sensor.value
            state, reward, isEnded, _ = self.environment.step(index)

            row, col, passenger, destination = self.decode(state)
            # print(state,"=",row,col,passenger,destination)

            self.sensor_row.update(row)
            self.sensor_col.update(col)
            self.sensor_passenger.update(passenger)
            self.sensor_destination.update(destination)
            self.state_sensor.update(state)
            # if self.isEnded and reward ==0:
            #    reward=-1

            self.reward_sensor.update(reward)
            self.reward_sensor.updates_evaluation(reward, isEnded)
            # print("ex: ", index, "last state",last_state,
            #      "new state",state,"with reward:",reward)

            if isEnded:
                state = self.environment.reset()
                row, col, passenger, destination = self.decode(state)
                self.sensor_row.update(row)
                self.sensor_col.update(col)
                self.sensor_passenger.update(passenger)
                self.sensor_destination.update(destination)
                self.state_sensor.update(state)
                # self.sensor.update(state)
            """self.state_sensor.sync()
            self.sensor_col.sync()
            self.sensor_row.sync()
            self.sensor_destination.sync()
            self.sensor_passenger.sync()
            self.reward_sensor.sync()"""
        except Exception as e:
            print(e)
            return


class BehaviorShell(BehaviourBase):
    """
    A simple behaviour for triggering generic MAPC actions that just need a type and static parameters
    """

    def __init__(self, index, name, params=[], **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param action_type: type of the MAC action
        :param params: optional parameters for the MAC action
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(BehaviorShell, self).__init__(name=name, requires_execution_steps=True, **kwargs)
        self.index = index
        print("init behavior", self.index)

    def do_step(self):
        return

    def unregister(self, terminate_services=True):
        super(BehaviorShell, self).unregister(terminate_services=terminate_services)


class LocationCondition(Condition):
    '''
    This is the basic Condition class it brings together the sensor and the activation function and takes care
    of the processing and caching of calculated data
    '''
    _instanceCounter = 0  # static _instanceCounter to get distinguishable names

    def __init__(self, sensor, activator, sensors, is_pick_up, name=None, optional=False):
        super(LocationCondition, self).__init__(sensor, activator, name, optional)
        self.sensor_row = sensors[0]
        self.sensor_col = sensors[1]
        self.sensor_passenger = sensors[2]
        self.sensor_destination = sensors[3]
        self.state_sensor = sensors[4]
        self.is_pick_up = is_pick_up

    def sync(self):
        self.sensor.sync()
        self.sensor_row.sync()
        self.sensor_col.sync()
        self.sensor_passenger.sync()
        self.sensor_destination.sync()
        self.state_sensor.sync()

    def updateComputation(self):
        '''
        This method needs to be executed at first on every decision cycle before accessing all other values/results/methods
        '''
        if self.sensor_passenger._value is None:
            self._satisfaction = 0.0
            return
        if self.sensor_col._value is None:
            self._satisfaction = 0.0
            return
        if self.sensor_destination._value is None:
            self._satisfaction = 0.0
            return
        if self.sensor_row._value is None:
            self._satisfaction = 0.0
            return

        locs = [(0, 0), (0, 4), (4, 0), (4, 3)]
        if self.is_pick_up:

            location = self.sensor_passenger.value
            if location == 4:
                self._satisfaction = 0.0
                return

        else:
            location = self.sensor_destination.value
        dest = locs[location]
        if self.sensor_row.value == dest[0] and self.sensor_col.value == dest[1]:
            self._satisfaction = 1.0
        else:
            self._satisfaction = 0.0


class RewardSensor(Sensor):
    """
    A simple behaviour for triggering generic MAPC actions that just need a type and static parameters
    """

    def __init__(self, name, intervall=10, **kwargs):
        super(RewardSensor, self).__init__(name=name, **kwargs)
        self.step = 0
        self.step_last = 0
        self.reward = 0
        self.reward_last = 0
        self.updates = 0
        self.intervall = intervall

    def updates_evaluation(self, reward, bool, ignore=False):
        # print("rewar dupdate")
        self.updates += 1
        if not ignore:
            self.reward += reward
            self.reward_last += reward
        if bool:
            self.step += 1
            self.step_last += 1
            if self.step_last == self.intervall:
                print(self.updates, self.intervall, self.step, "average_rate:",
                      numpy.round(self.reward / float(self.step), 3), "last_rate:",
                      self.reward_last / float(self.step_last))
                self.reward_last = 0
                self.step_last = 0
