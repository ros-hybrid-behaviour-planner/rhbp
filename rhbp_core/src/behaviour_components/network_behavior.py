'''
Created on 13.04.2015

@author: rieger
'''

from .activators import Activator, BooleanActivator, Condition
from .behaviours import BehaviourBase
from .goals import OfflineGoal
from .managers import Manager


class NetworkBehavior(BehaviourBase):

    MANAGER_POSTFIX = "/Manager"

    '''
    Behavior, which contains other behaviors
    Must be in separate file, because of circular dependencies (depends on manager, but manager depends on Behavior)
    '''

    def __init__(self, effects, name, requires_execution_steps=False, **kwargs):
        '''

        :param effects: tuple <sensor,Effect>
        :param name:
        :param requires_execution_steps:
        :param kwargs:
        '''
        super(NetworkBehavior, self).__init__(name=name, requires_execution_steps=requires_execution_steps,
                                              kwargs=kwargs)
        self.requires_execution_steps = requires_execution_steps
        managerArgs = {}
        managerArgs.update(kwargs)
        managerArgs['prefix'] = name + NetworkBehavior.MANAGER_POSTFIX
        self.__manager = Manager(activated=False, **managerArgs)

        self.__goal_name_prefix = name + "/Goals/"
        self.__goal_counter = 0

        correlations = []
        for effect in effects:
            correlations.append(effect[1])
            goal_name = self.__generate_goal_name(effect[1])
            activator_name = self._restore_condition_name_from_pddl_function_name(effect[1].sensorName, effect[0].name)
            goal = self._create_goal(sensor=effect[0], effect=effect[1], goal_name=goal_name,
                                     activator_name=activator_name)
            self.__manager.add_goal(goal)
        self.correlations=correlations

    def _restore_condition_name_from_pddl_function_name(self, pddl_function_name, sensor_name):
        return Activator.restore_condition_name_from_pddl_function_name(pddl_function_name=pddl_function_name,
                                                                        sensor_name=sensor_name)

    def __generate_goal_name(self, effect):
        '''
        :param effect: instance of type  Effect
        :return: unique name for goal
        '''
        # x as seperator between counter an sensor names, to prevent conflict not suitable names
        name = self.__goal_name_prefix + str(self.__goal_counter) + 'X' + effect.sensorName
        self.__goal_counter += 1
        return name

    def _create_goal(self, sensor, effect, goal_name, activator_name):
        '''
        :param sensor: instance of type Sensor
        :param effect: instance of type  Effect
        :param goal_name: unique name for the goal
        :return: a goal with a greedy condition
        '''
        if (effect.sensorType == bool):
            desired_value = True if effect.indicator > 0 else False
            activator = BooleanActivator(name=activator_name, desiredValue=desired_value)
            condition = Condition(activator=activator, sensor=sensor)
            return OfflineGoal(goal_name, permanent=True, conditions={condition})

    def do_step(self):
        self.__manager.step()

    def start(self):
        self.__manager.activate()

    def stop(self):
        self.__manager.deactivate()
