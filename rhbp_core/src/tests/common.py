'''
Common implementations (behaviours, sensors) for reuse in several tests

Created on 27.03.2017

@author: hrabia, rieger
'''

import rospy

from std_msgs.msg import Bool,Int32

from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect


class SetTrueBehavior(BehaviourBase):
    """
    Behavior, which publishes True in the given topic
    """

    def __init__(self, effect_name, topic_name, name, **kwargs):
        super(SetTrueBehavior, self).__init__(name, requires_execution_steps=True, **kwargs)
        if effect_name:
            self._correlations = [Effect(sensor_name=effect_name, indicator=1, sensor_type=bool)]
        self.__publisher = rospy.Publisher(topic_name, Bool, queue_size=10)
        self.was_executed = False
        rospy.sleep(0.1)

    def start(self):
        super(SetTrueBehavior, self).start()
        rospy.loginfo("%s started", self._name)

    def stop(self):
        self.was_executed = True
        rospy.loginfo("%s stopped", self._name)

    def do_step(self):
        rospy.loginfo("%s stepped", self._name)
        self.__publisher.publish(True)


class IncreaserBehavior(BehaviourBase):
    """
    Behavior, which increases an int value in the given topic
    """

    def __init__(self, effect_name, topic_name, name='increaserAdderBehaviour',
                 **kwargs):
        super(IncreaserBehavior, self).__init__(name, requires_execution_steps=True, **kwargs)
        if effect_name:
            self._correlations = [Effect(sensor_name=effect_name, indicator=1, sensor_type=int)]
        self.__publisher = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.__next_value = 1
        self.was_executed = False
        rospy.sleep(0.1)

    def start(self):
        # we override this function here in order to avoid initial do_step() execution
        rospy.loginfo("%s started", self._name)
        pass

    def stop(self):
        rospy.loginfo("%s stopped", self._name)
        self.was_executed = True

    def do_step(self):
        rospy.loginfo("%s stepped", self._name)
        self.__publisher.publish(Int32(self.__next_value))
        self.__next_value += 1