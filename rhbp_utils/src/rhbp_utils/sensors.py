"""
Created on 17.04.2018

@author: hrabia
"""

from behaviour_components.sensors import AggregationSensor, RawTopicSensor

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.utils')


class ValueDifferSensor(AggregationSensor):
    """
    Aggregation Sensor returning `True` if the two sensor values differ (or no values are available), otherwise `False`
    """

    def __init__(self, name, sensors, publish_aggregate=False, optional=False, initial_value=None):
        """
        :param sensors: list of other sensors to aggregate
        :param publish_aggregate: if set to true the aggregated value will be published on a ROS topic, which name can
               be retrieved with topic_name property
        :return should be an aggregated and normalised function value
        """
        super(ValueDifferSensor, self).__init__(name=name, sensors=sensors, publish_aggregate=publish_aggregate,
                                                    optional=optional, initial_value=initial_value, func=None)

    def _aggregate(self, sensor_values):

        value = True

        if sensor_values[0] is not None and sensor_values[1] is not None:

            if str(sensor_values[0]) == str(sensor_values[1]):
                value = False

        elif sensor_values[0] is None and sensor_values[1] is None:
            value = False

        return value


class GoalAmountSensor(RawTopicSensor):
    """
    Sensor that determines the number of currently registered goals in the planner
    """

    def __init__(self, topic="/Planner/plannerStatus", name=None, initial_value=0, create_log=False,
                 print_updates=False):
        super(GoalAmountSensor, self).__init__(name=name, topic=topic, initial_value=initial_value,
                                               create_log=create_log, print_updates=print_updates)

    def update(self, value):

        if value.goals:
            amount_of_goals = len(value.goals)
        else:
            amount_of_goals = 0

        super(GoalAmountSensor, self).update(amount_of_goals)


class GoalEnabledAmountSensor(RawTopicSensor):
    """
    Sensor that determines the number of currently registered and enabled goals in the planner.
    Moreover, it allows to filter permanent and non-permanent goals.
    """

    def __init__(self, topic="/Planner/plannerStatus", name=None, initial_value=0, permanent_to_filter=None,
                 create_log=False, print_updates=False):
        """
        :param permanent_to_filter: None = No filter, True = only permanent (maintenance goals),
        False = only non-permanent, goals (achievement goals)
        """
        super(GoalEnabledAmountSensor, self).__init__(name=name, topic=topic, initial_value=initial_value,
                                                      create_log=create_log, print_updates=print_updates)
        self.filter_permanent = permanent_to_filter

    def update(self, value):

        if value.goals:
            if self.filter_permanent is None:
                amount_of_goals = len([g for g in value.goals if g.enabled])
            else:
                amount_of_goals = len([g for g in value.goals if g.enabled and g.permanent == self.filter_permanent])
        else:
            amount_of_goals = 0

        super(GoalEnabledAmountSensor, self).update(amount_of_goals)


class GoalEnabledWithNameSensor(RawTopicSensor):
    """
    Sensor that determines if a goal containing the targeted name is enabled and registered.
    """

    def __init__(self, target_goal_name, topic="/Planner/plannerStatus", name=None, initial_value=False,
                 create_log=False, print_updates=False):
        """
        :param target_goal_name: name component that is searched with "in"
        """
        super(GoalEnabledWithNameSensor, self).__init__(name=name, topic=topic, initial_value=initial_value,
                                                        create_log=create_log, print_updates=print_updates)
        self.target_goal_name = target_goal_name

    def update(self, value):

        goal_found = False

        for g in value.goals:
            if self.target_goal_name in g.name and g.enabled:
                goal_found = True
                break

        super(GoalEnabledWithNameSensor, self).update(goal_found)
