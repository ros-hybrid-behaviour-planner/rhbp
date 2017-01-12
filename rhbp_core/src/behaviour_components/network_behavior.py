'''
Created on 13.04.2015

@author: rieger
'''

from .activators import Activator, BooleanActivator, Condition, GreedyActivator
from .behaviours import BehaviourBase
from .goals import OfflineGoal
from .managers import Manager


class NetworkBehavior(BehaviourBase):
    '''
    Behavior, which contains other behaviors
    Must be in separate file, because of circular dependencies (depends on manager, but manager depends on Behavior)
    '''

    MANAGER_POSTFIX = "/Manager"

    def __init__(self, effects, name, requires_execution_steps=False, **kwargs):
        '''

        :param effects: tuple <sensor,Effect>
        :param name:
        :param requires_execution_steps: whether the execution steps should be caused from the parent manager or not.
                If not, the step method must be called manually
        :param kwargs: args for the manager, except the prefix arg
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
        self.correlations = correlations

    def _restore_condition_name_from_pddl_function_name(self, pddl_function_name, sensor_name):
        return Activator.restore_condition_name_from_pddl_function_name(pddl_function_name=pddl_function_name,
                                                                        sensor_name=sensor_name)

    def __generate_goal_name(self, effect):
        '''
        :param effect: instance of type  Effect
        :return: unique name for goal
        '''
        # x as seperator between counter an sensor names, to prevent conflict, caused by unusual names
        name = self.__goal_name_prefix + str(self.__goal_counter) + 'X' + effect.sensorName
        self.__goal_counter += 1
        return name

    def _create_goal(self, sensor, effect, goal_name, activator_name):
        '''
        :param sensor: instance of type Sensor
        :param effect: instance of type  Effect
        :param goal_name: unique name for the goal
        :return: a goal, which causes the manager to work on the ffect during the whole time
        '''
        if (effect.sensorType == bool):
            desired_value = True if effect.indicator > 0 else False
            activator = BooleanActivator(name=activator_name, desiredValue=desired_value)
            condition = Condition(activator=activator, sensor=sensor)
            return OfflineGoal(goal_name, permanent=True, conditions={condition})
        if (effect.sensorType == int or effect.sensorType == float or effect.sensorType == long):
            activator = GreedyActivator(maximize=effect.indicator > 0, step_size=effect.realWorldImpact,
                                        name=activator_name)
            condition = Condition(activator=activator, sensor=sensor)
            return OfflineGoal(goal_name, permanent=True, conditions={condition})
        raise RuntimeError(msg='Cant create goal for effect type \'' + str(
            effect.sensorType) + '\'. Overwrite the method _create_goal for handle the type')

    def do_step(self):
        self.__manager.step()

    def start(self):
        self.__manager.activate()

    def stop(self):
        self.__manager.deactivate()
