'''
Common implementations (behaviours, sensors) for reuse in several tests

Created on 27.03.2017

@author: hrabia, rieger
'''

import rospy

from std_msgs.msg import Bool,Int32

from behaviour_components.behaviours import BehaviourBase
from behaviour_components.pddl import Effect


class SetTrueBehavior(BehaviourBase):
    """
    Behavior, which publishes True in the given topic
    """

    def __init__(self, effect_name, topic_name, name, **kwargs):
        super(SetTrueBehavior, self).__init__(name, requires_execution_steps=True, **kwargs)
        self._correlations = [Effect(effect_name, 1, sensorType=bool)]
        self.__publisher = rospy.Publisher(topic_name, Bool, queue_size=10)
        self.was_executed = False
        rospy.sleep(0.1)

    def stop(self):
        self.was_executed = True

    def do_step(self):
        self.__publisher.publish(True)


class IncreaserBehavior(BehaviourBase):
    """
    Behavior, which increases an int value in the given topic
    """

    def __init__(self, effect_name, topic_name, name='increaserAdderBehaviour',
                 **kwargs):
        super(IncreaserBehavior, self).__init__(name, requires_execution_steps=True, **kwargs)
        self._correlations = [Effect(effect_name, 1, sensorType=int)]
        self.__publisher = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.__next_value = 1
        self.was_executed = False
        rospy.sleep(0.1)

    def start(self):
        # we override this function here in order to avoid initial do_step() execution
        pass

    def stop(self):
        self.was_executed = True

    def do_step(self):
        self.__publisher.publish(Int32(self.__next_value))
        self.__next_value += 1