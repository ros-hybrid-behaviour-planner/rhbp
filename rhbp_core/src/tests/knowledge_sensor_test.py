#!/usr/bin/env python
import unittest

import rospy
import time
import rostest
from behaviour_components.sensors import KnowledgeSensor
from knowledge_base.msg import Push
from knowledge_base.srv import Pop

PKG = 'rhbp_core'


class TestKnowledgeBaseSensor(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestKnowledgeBaseSensor, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix = 'TupleSpaceTestSuite' + str(time.time())

    @staticmethod
    def add_tupple(tuple):
        pub = rospy.Publisher('/knowledgeBaseNode/Push', Push, queue_size=10)
        rospy.sleep(1)
        pub.publish(tuple)

    def test_basic(self):
        sensor = KnowledgeSensor(pattern=((self.__message_prefix, 'test_basic', 'pos', '*', '*')))
        sensor.sync()
        self.assertFalse(sensor.value)

        TestKnowledgeBaseSensor.add_tupple((self.__message_prefix, 'test_basic', 'pos', '42', '0'))
        rospy.sleep(1)

        sensor.sync()
        self.assertTrue(sensor.value)

    def test_remove(self):

        test_tuple= (self.__message_prefix, 'test_remove', 'pos', '42', '0')
        TestKnowledgeBaseSensor.add_tupple(test_tuple)
        rospy.sleep(1)

        sensor = KnowledgeSensor(pattern=((self.__message_prefix, 'test_remove', 'pos', '*', '*')))
        sensor.sync()
        self.assertTrue(sensor.value)


        rospy.wait_for_service('/knowledgeBaseNode/Pop')
        pop_service = rospy.ServiceProxy('/knowledgeBaseNode/Pop', Pop)
        pop_response = pop_service(test_tuple)

        sensor.sync()
        self.assertFalse(sensor.value)


if __name__ == '__main__':
    rospy.init_node('knowledge_sensor_test_node', log_level=rospy.DEBUG)
    rostest.rosrun(PKG, 'knowledge_sensor_test_node', TestKnowledgeBaseSensor)
