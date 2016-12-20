#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: phillip
'''
import time
import unittest
import rospy
import rostest
from knowledge_base.srv import Exists, Peek, Pop, All
from knowledge_base.msg import Push


PKG = 'knowledge_base'

"""
System test for knowledge base. Assumes, that a rosmaster and the knowledge base is running
"""
class TupleSpaceTestSuite(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TupleSpaceTestSuite, self).__init__(*args, **kwargs)
        # prevent influence of previous tests
        self.__message_prefix='TupleSpaceTestSuite' + str(time.time())

    @staticmethod
    def add_tupple(tuple):
        pub = rospy.Publisher('/knowledgeBaseNode/Push', Push, queue_size=10)
        rospy.sleep(1)
        pub.publish(tuple)

    def test_exists_for_non_existing(self):
        test_tuple = (self.__message_prefix, 'test_exists_for_non_existing','0','0')
        rospy.wait_for_service('/knowledgeBaseNode/Exists')
        exist_service= rospy.ServiceProxy('/knowledgeBaseNode/Exists',Exists)
        self.assertFalse(exist_service(test_tuple).exists)

    def test_simple_adding(self):
        test_tuple = (self.__message_prefix,'test_simple_adding','0','0')
        self.add_tupple(test_tuple)
        rospy.wait_for_service('/knowledgeBaseNode/Exists')
        exist_service= rospy.ServiceProxy('/knowledgeBaseNode/Exists',Exists)
        self.assertTrue(exist_service(test_tuple))

    def test_peek(self):
        test_tuple = (self.__message_prefix,'test_peek','0','0')
        self.add_tupple(test_tuple)
        peekService= rospy.ServiceProxy('/knowledgeBaseNode/Peek',Peek)
        peekResponse= peekService(test_tuple)
        self.assertTrue(True,peekResponse.exists)
        self.assertEqual(test_tuple,tuple(peekResponse.example))

    def test_pop(self):
        test_tuple = (self.__message_prefix,'test_pop','0','0')
        self.add_tupple(test_tuple)
        rospy.wait_for_service('/knowledgeBaseNode/Exists')
        exist_service= rospy.ServiceProxy('/knowledgeBaseNode/Exists',Exists)
        self.assertTrue(exist_service(test_tuple).exists)

        rospy.wait_for_service('/knowledgeBaseNode/Pop')
        pop_service = rospy.ServiceProxy('/knowledgeBaseNode/Pop', Pop)
        pop_response = pop_service(test_tuple)
        self.assertTrue(pop_response.exists)
        self.assertEqual(test_tuple,tuple(pop_response.removed))

        self.assertFalse(exist_service(test_tuple).exists)

    def test_placeholder(self):
        test_tuple = (self.__message_prefix,'test_placeholder','0','0')
        pattern = (self.__message_prefix,'test_placeholder','*','*')

        self.add_tupple(test_tuple)
        peek_service= rospy.ServiceProxy('/knowledgeBaseNode/Peek',Peek)
        peek_response= peek_service(pattern)
        self.assertTrue(peek_response.exists)
        self.assertEqual(test_tuple,tuple(peek_response.example))

    def __wait_for_tuple(self,wait_for_it):
        """
        waits, until the requested tuple is contained in knowledge_base
        :param wait_for_it: tuple
        """
        rospy.wait_for_service('/knowledgeBaseNode/Exists')
        exist_service= rospy.ServiceProxy('/knowledgeBaseNode/Exists',Exists)
        while not exist_service(wait_for_it).exists:
            rospy.sleep(1)

    def __is_tuple_in_facts(self,to_check,facts):
        """
        :param to_check: tuple
        :param facts: tuple of ROS message Fact
        :return: whether tuple is contained in content of at least one fact
        """
        for current_tuple in facts:
            if tuple(current_tuple.content)==to_check:
                return True
        return False

    def test_all(self):
        t1 = (self.__message_prefix,'test_all','pos','0','0')
        self.add_tupple(t1)
        t2 = (self.__message_prefix,'test_all','pos','1','0')
        self.add_tupple(t2)
        t3 = (self.__message_prefix,'test_all','pos','1','-4')
        self.add_tupple(t3)

        self.__wait_for_tuple(t3)

        rospy.wait_for_service('/knowledgeBaseNode/All')
        all_service= rospy.ServiceProxy('/knowledgeBaseNode/All',All)
        all_response=all_service((self.__message_prefix,'test_all','pos','*','*'))

        self.assertEqual(3,len(all_response.found))
        self.assertTrue(self.__is_tuple_in_facts(t1,all_response.found))
        self.assertTrue(self.__is_tuple_in_facts(t2,all_response.found))
        self.assertTrue(self.__is_tuple_in_facts(t3,all_response.found))

    def test_prevent_multiple_adding(self):
        test_tuple = (self.__message_prefix,'test_prevent_multiple_adding','0','0')
        self.add_tupple(test_tuple)
        self.add_tupple(test_tuple)

        rospy.wait_for_service('/knowledgeBaseNode/All')
        all_service= rospy.ServiceProxy('/knowledgeBaseNode/All',All)
        all_response=all_service(test_tuple)
        self.assertEqual(1,len(all_response.found))




if __name__ == '__main__':
    rospy.init_node('TupleSpaceTestSuite', log_level=rospy.DEBUG)
    rostest.rosrun(PKG, 'knowledge_base_manager_test_node', TupleSpaceTestSuite)