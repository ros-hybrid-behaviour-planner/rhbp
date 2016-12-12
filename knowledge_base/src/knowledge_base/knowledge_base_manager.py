#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: phillip
'''

import rospy
import sys

from tuple_space import TupleSpace
from knowledge_base.srv import Exists, Peek, PeekResponse, Pop, PopResponse, All, AllResponse
from knowledge_base.msg import Push, Fact

"""
Wrapper class for accessing the real tupple space
"""
class KnowledgeBase(object):


    def __init__(self, name):
        self.__tuple_space = TupleSpace()
        rospy.Subscriber(name + '/Push', Push, self.__push)
        self.__exists_service = rospy.Service(name + '/Exists', Exists, self.__exists)
        self.__peek_service = rospy.Service(name + '/Peek', Peek, self.__peek)
        self.__pop_service = rospy.Service(name + '/Pop', Pop, self.__pop)
        self.__all_service = rospy.Service(name+'/All',All, self.__all)

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
        lst = list(pattern)
        for i in range(0, len(lst)):
            if (lst[i] == '*'):
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
        if not self.__exists_tupple_as_is(converted):
            self.__tuple_space.add(converted)

    def __exists_tupple_as_is(self, tuple):
        """
        Checks whether the tuple is returned in tuple space. Just a wrapper method.
        Does no conversion of the requested tuple
        :return: whether the requested tuple is contained in the tuple space.
        """
        try:
            self.__tuple_space.get(tuple)
            return True
        except KeyError:
            return False

    def __exists(self, request):
        """
        :param request: Exists, as defined as ROS service
        :return: bool, which indiciated, whether a tupple exists in knowledge base, which matchs the pattern
        """
        converted = self.__converts_request_to_tuple_space_format(request.pattern)
        return self.__exists_tupple_as_is(converted)

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
        except KeyError:
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
        except KeyError:
            return PopResponse(exists=False)

    def __all(self,allRequest):
        """
        :return: all contained tupples, matching the given pattern, but never more tuples than sys.max_int
        """
        converted = self.__converts_request_to_tuple_space_format(allRequest.pattern)
        try:
            foundTuples = self.__tuple_space.many(converted,sys.maxint)
            resultAsList= []
            for fact in foundTuples:
                resultAsList.append(Fact(content=fact[1]))
            return AllResponse(found=tuple(resultAsList))
        except KeyError:
            return ()
