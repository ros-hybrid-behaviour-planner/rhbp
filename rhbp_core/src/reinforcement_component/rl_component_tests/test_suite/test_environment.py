import time

import numpy

from behaviour_components.behaviours import BehaviourBase
from behaviour_components.sensors import Sensor


class TestBaseEnvironment(object):
    def __init__(self,env,manager,sensors):
        self.env = env
        #self.env.seed(0)
        numpy.random.seed(0)
        self.manager = manager
        self.sensors = sensors
        self.frequency = 0.01
        self.count = 0
        self.last_none=True

    def behaviour_to_index(self,name):
        num = 0
        for b in self.manager.behaviours:
            if b == name:
                return num
            num += 1
        return None

    def start_simulation(self):
        while(True):
            time.sleep(self.frequency)
            self.count +=1

            #if self.count % 100 == 0:
            #print(self.count)
            #print(self.manager.behaviours)
            self.manager.step()

            executed_behaviors = self.manager.executedBehaviours
            num_actions = len(executed_behaviors)
            # check if no action was selected. Then repeat this cycle
            if num_actions>0:
                #ex_index = executed_behaviors[0].index
                if self.last_none:
                    self.last_none=False
                else:
                    ex_index=self.behaviour_to_index(executed_behaviors[0])
                    self.execute_step(ex_index)
            else:
                self.last_none = True
    def execute_step(self,index):
        raise NotImplementedError

class TestFrozenLakeEnv(TestBaseEnvironment):
    def __init__(self,env,manager,sensors):
        super(TestFrozenLakeEnv, self).__init__(env,manager,sensors)
        self.sensor = sensors[0]
        self.reward_sensor = sensors[1]
        self.last_state = 0
    def execute_step(self,index):
        #print("execute step",index)
        #last_state = self.sensor.value
        state,reward,isEnded, _ = self.env.step(index)

        if isEnded and reward ==0:
            reward=-1

        print("ex: ", index, "last state", self.last_state,
              "new state",state,"with reward:",reward)

        self.reward_sensor.update(reward)
        if reward == -1:
            self.reward_sensor.updates_evaluation(reward,isEnded,ignore=True)
        else:
            self.reward_sensor.updates_evaluation(reward, isEnded)
        #print("ex: ", self.index, "last state",last_state,
        #      "new state",state,"with reward:",reward)

        if isEnded:
            state = self.env.reset()

        self.sensor.update(state)

        self.last_state = state

class BehaviorShell(BehaviourBase):
    """
    A simple behaviour for triggering generic MAPC actions that just need a type and static parameters
    """

    def __init__(self,index,name, params=[], **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param action_type: type of the MAC action
        :param params: optional parameters for the MAC action
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(BehaviorShell, self).__init__(name=name, requires_execution_steps=True, **kwargs)
        self.index = index
        print("init behavior",self.index)

    def do_step(self):
        return

    def unregister(self, terminate_services=True):
        super(BehaviorShell, self).unregister(terminate_services=terminate_services)


class RewardSensor(Sensor):
    """
    A simple behaviour for triggering generic MAPC actions that just need a type and static parameters
    """

    def __init__(self,name,intervall=10, **kwargs):
        super(RewardSensor, self).__init__(name=name, **kwargs)
        self.step =0
        self.step_last=0
        self.reward = 0
        self.reward_last = 0
        self.updates=0
        self.intervall=intervall
    def updates_evaluation(self,reward,bool,ignore=False):
        #print("rewar dupdate")
        self.updates +=1
        if not ignore:
            self.reward += reward
            self.reward_last += reward
        if bool:
            self.step += 1
            self.step_last += 1
            if self.step_last == self.intervall:
                print(self.updates,"average_rate:",numpy.round(self.reward/float(self.step),3),"last_rate:",self.reward_last/float(self.step_last))
                self.reward_last=0
                self.step_last=0