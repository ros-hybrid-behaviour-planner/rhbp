#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: phillip
'''

import rospy

from tuple_space import TupleSpace
from knowledge_base.srv import Exists, Peek, PeekResponse, Pop, PopResponse
from knowledge_base.msg import Push


class KnowledgeBase(object):
"""
Wrapper class for accessing the real tupple space
"""

    def __init__(self, name):
        self.__tuple_space = TupleSpace()
        rospy.Subscriber(name + '/Push', Push, self.__push)
        self.__exists_service = rospy.Service(name + '/Exists', Exists, self.__exists)
        self.__peek_service = rospy.Service(name + '/Peek', Peek, self.__peek)
        self.__pop_service = rospy.Service(name + '/Pop', Pop, self.__pop)

    def __del__(self):
        """
            closes all services
        """
        self.__exists_service.shutdown()
        self.__peek_service.shutdown()
        self.__pop_service.shutdown()

    def __converts_request_to_tuple_space_format(self, pattern):
        """
        :param pattern:  tupple of non-None strings
        :return: tupple. At each position in the source, where the placeholder * was, is now the typ str
        """
        #TODO: use * for placeholder detection
        lst = list(pattern)
        for i in range(0, len(lst)):
            if (lst[i] == None):
                lst[i] = str
        return tuple(lst)

    def __push(self, request):
        """
        Adds the fact to the tupple space
        :param request: Push message, as defined as ROS message
        """
        # Since all read request converts nones to string type, it must be done also here.
        # Otherwise the stored tupple can't readed or removed anymore
        converted = self.__converts_request_to_tuple_space_format(request.content)
        self.__tuple_space.add(converted)

    def __exists(self, request):
        """
        :param request: Exists, as defined as ROS service
        :return: bool, which indiciated, whether a tupple exists in knowledge base, which matchs the pattern
        """
        try:
            converted = self.__converts_request_to_tuple_space_format(request.pattern)
            self.__tuple_space.get(converted)
            return True
        except KeyError as e:
            return False

    def __peek(self, request):
        """
        :param request: Peek, as defined as ROS service
        :return: PeekResponse, as defined as ROS service respone
                if a tupple exists in tupple space, an example is returned and the exists flag is setted
        """
        try:
            converted = self.__converts_request_to_tuple_space_format(request.pattern)
            result = self.__tuple_space.get(converted)
            return PeekResponse(example=result, exists=True)
        except KeyError as e:
            return PeekResponse(exists=False)

    def __pop(self, request):
        """
        If tupple exists in the tupple space, which matchs the pattern, it will be also removed from  the tupple space
        and returned
        :param request: Pop, as defined as ROS service
        :return: PopResponse, as defined as ROS service respone
                if a tupple exists in tupple space, an example is returned and the exists flag is setted
        """
        try:
            converted = self.__converts_request_to_tuple_space_format(request.pattern)
            result = self.__tuple_space.get(converted, remove=True)
            return PopResponse(removed=result, exists=True)
        except KeyError as e:
            return PopResponse(exists=False)
