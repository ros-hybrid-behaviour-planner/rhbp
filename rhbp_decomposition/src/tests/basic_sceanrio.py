

import rospy

from behaviour_components.activators import ThresholdActivator, BooleanActivator
from behaviour_components.conditions import Condition
from behaviour_components.goals import OfflineGoal
from behaviour_components.network_behavior import NetworkBehaviour
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect
from behaviour_components.sensors import TopicSensor

from std_msgs.msg import Bool, Int32

# TODO is this necessary? better remove this

class TakeOrderBehaviour(BehaviourBase):

    def __init__(self, topic_name, name, **kwargs):
        super(TakeOrderBehaviour, self).__init__(name, requires_execution_steps=True, **kwargs)
        self.__publisher = rospy.Publisher(topic_name, Bool, queue_size=10)
        self.__progress = 0

    def start(self):
        super(TakeOrderBehaviour, self).start()
        rospy.loginfo("%s started", self._name)
        self.__progress = 0

    def stop(self):
        rospy.loginfo("%s stopped", self._name)

    def do_step(self):
        rospy.loginfo("%s stepped", self._name)
        self.__publisher.publish(True)
        self.__progress += 1

    def getProgress(self):
        return self.__progress


class IncrementalBehaviour(BehaviourBase):

    def __init__(self, topic_name, name, increment=1, starting_value=0, **kwargs):
        super(IncrementalBehaviour, self).__init__(name, requires_execution_steps=True, **kwargs)
        self.__publisher = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.__increment = increment
        self.__current_value = starting_value
        self.__starting_value = starting_value

    def start(self):
        super(IncrementalBehaviour, self).start()
        self.__current_value = self.__starting_value
        rospy.loginfo("%s started", self._name)

    def stop(self):
        rospy.loginfo("%s stopped", self._name)

    def do_step(self):
        rospy.loginfo("%s stepped", self._name)
        self.__current_value += self.__increment
        self.__publisher.publish(self.__current_value)


class BasicCookingRobot(object):
    """


    """

    def __init__(self, planner_prefix='BPR'):
        self.__prefix = planner_prefix

        name = planner_prefix+'/'+"TakeOrder"
        topic_name = self.__prefix + '/' + name
        sensor_name = name + '/Sensor'
        take_order_behaviour = TakeOrderBehaviour(name=name, topic_name=topic_name, executionTimeout=2,
                                                  plannerPrefix=self.__prefix)
        take_order_sensor = TopicSensor(name=sensor_name, topic=topic_name, message_type=Bool, initial_value=False)
        take_order_behaviour.add_effect(Effect(sensor_name=sensor_name, indicator=1, sensor_type=bool))
        take_order_cond = Condition(take_order_sensor, BooleanActivator())

        cook_behaviour = NetworkBehaviour(name=planner_prefix+'/'+'Cook', plannerPrefix=self.__prefix, createLogFiles=True)
        cook_behaviour.add_precondition(take_order_cond)

        deliver_behaviour = NetworkBehaviour(name=planner_prefix+'/'+'Deliver', plannerPrefix=self.__prefix, createLogFiles=True)

        name = planner_prefix+'/'+"MakeDough"
        topic_name = cook_behaviour.get_manager_prefix() + '/' + name
        sensor_name = name + '/Sensor'
        make_dough_behaviour = IncrementalBehaviour(topic_name=topic_name, name=name, executionTimeout=15,
                                                    plannerPrefix=cook_behaviour.get_manager_prefix())
        make_dough_sensor = TopicSensor(name=sensor_name, topic=topic_name, message_type=Int32, initial_value=0)
        make_dough_behaviour.add_effect(Effect(sensor_name=sensor_name, indicator=1, sensor_type=int))
        make_dough_cond = Condition(make_dough_sensor, ThresholdActivator(thresholdValue=10))

        name = planner_prefix+'/'+"MakeSauce"
        topic_name = cook_behaviour.get_manager_prefix() + '/' + name
        sensor_name = name + '/Sensor'
        make_sauce_behaviour = IncrementalBehaviour(name=name, topic_name=topic_name, executionTimeout=10,
                                                    increment=1, plannerPrefix=cook_behaviour.get_manager_prefix())
        make_sauce_sensor = TopicSensor(name=sensor_name, topic=topic_name, message_type=Int32, initial_value=0)
        make_sauce_behaviour.add_effect(Effect(sensor_name=sensor_name, indicator=1, sensor_type=int))
        make_sauce_cond = Condition(make_sauce_sensor, ThresholdActivator(thresholdValue=5))

        name = planner_prefix+'/'+"MakePizza"
        topic_name = cook_behaviour.get_manager_prefix() + '/' + name
        sensor_name = name + '/Sensor'
        make_pizza_behaviour = IncrementalBehaviour(name=name, plannerPrefix=cook_behaviour.get_manager_prefix(),
                                                    topic_name=topic_name, executionTimeout=10)
        make_pizza_behaviour.add_precondition(make_dough_cond)
        make_pizza_behaviour.add_precondition(make_sauce_cond)
        make_pizza_sensor = TopicSensor(name=sensor_name, topic=topic_name, message_type=Int32, initial_value=0)
        make_pizza_behaviour.add_effect(Effect(sensor_name=sensor_name, indicator=1, sensor_type=int))
        make_pizza_cond = Condition(make_pizza_sensor, ThresholdActivator(thresholdValue=5))

        # first level (cook_behaviour) goals and effects
        cook_behaviour.add_effect(Effect(sensor_name=sensor_name, indicator=5, sensor_type=int))
        cook_goal = OfflineGoal(name=planner_prefix+'/'+"CookGoal", planner_prefix=cook_behaviour.get_manager_prefix())
        cook_goal.add_condition(make_pizza_cond)
        cook_behaviour.add_goal(cook_goal)

        # deliver_behaviour has precondition
        deliver_behaviour.add_precondition(make_pizza_cond)

        name = planner_prefix+'/'+"PackUp"
        topic_name = cook_behaviour.get_manager_prefix() + '/' + name
        sensor_name = name + '/Sensor'
        pack_up_pizza_behaviour = IncrementalBehaviour(name=name, plannerPrefix=deliver_behaviour.get_manager_prefix(),
                                                       topic_name=topic_name, executionTimeout=10)
        pack_up_pizza_sensor = TopicSensor(name=sensor_name, topic=topic_name, message_type=Int32, initial_value=0)
        pack_up_pizza_behaviour.add_effect(Effect(sensor_name=sensor_name, indicator=1, sensor_type=int))
        pack_up_pizza_cond = Condition(pack_up_pizza_sensor, ThresholdActivator(thresholdValue=5))

        name = planner_prefix+'/'+"Transport"
        topic_name = cook_behaviour.get_manager_prefix() + '/' + name
        sensor_name = name + '/Sensor'
        transport_pizza_behaviour = IncrementalBehaviour(name=name, topic_name=topic_name, executionTimeout=20,
                                                         plannerPrefix=deliver_behaviour.get_manager_prefix())
        transport_pizza_behaviour.add_precondition(pack_up_pizza_cond)
        transport_pizza_sensor = TopicSensor(name=sensor_name, topic=topic_name, message_type=Int32,
                                             initial_value=0)
        transport_pizza_behaviour.add_effect(Effect(sensor_name=sensor_name, indicator=1, sensor_type=int))
        transport_pizza_cond = Condition(transport_pizza_sensor, ThresholdActivator(thresholdValue=15))

        name = planner_prefix+'/'+"CustomerInteraction"
        topic_name = cook_behaviour.get_manager_prefix() + '/' + name
        sensor_name = planner_prefix+'/'+"Got Money"
        customer_interaction_behaviour = TakeOrderBehaviour(name=name, topic_name=topic_name, executionTimeout=2,
                                                            plannerPrefix=deliver_behaviour.get_manager_prefix())
        customer_interaction_behaviour.add_precondition(transport_pizza_cond)
        customer_interaction_sensor = TopicSensor(name=sensor_name, topic=topic_name, message_type=Bool,
                                                  initial_value=False)
        customer_interaction_behaviour.add_effect(Effect(sensor_name=sensor_name, indicator=1, sensor_type=bool))
        customer_interaction_cond = Condition(customer_interaction_sensor, BooleanActivator())

        # set goal/effect for deliver behaviour
        deliver_behaviour.add_effect(Effect(sensor_name=sensor_name, indicator=1))
        deliver_goal = OfflineGoal(name=planner_prefix+'/'+"DeliverGoal", planner_prefix=deliver_behaviour.get_manager_prefix())
        deliver_goal.add_condition(customer_interaction_cond)
        deliver_behaviour.add_goal(deliver_goal)

        # set global goal
        self.__global_goal = OfflineGoal(name=planner_prefix+'/'+"MoneyGoal", planner_prefix=self.__prefix)
        self.__global_goal.add_condition(customer_interaction_cond)

        # test_goal
        self.__test_goal = OfflineGoal(name=planner_prefix+'/'+"TestGoal", planner_prefix=self.__prefix)
        self.__test_goal.add_condition(make_pizza_cond)

        # test condition
        self.__test_condition = make_pizza_cond

    def get_goal(self):
        return self.__global_goal

    def get_test_goal(self):
        return self.__test_goal

    def get_test_cond(self):
        return self.__test_condition

