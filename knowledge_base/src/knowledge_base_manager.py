#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: phillip
'''

import rospy
from knowledge_base.srv._Peek import Peek

from tuple_space import TupleSpace
from knowledge_base.srv import Exists, ExistsResponse, Peek, PeekResponse, Pop, PopResponse
from knowledge_base.msg import Push


class KnowledgeBase(object):


    def __init__(self, name):
        self.__tuple_space = TupleSpace()
        rospy.Subscriber(name + '/Push', Push, self.__push)
        self.__exists_service = rospy.Service(name + '/Exists', Exists, self.__exists)
        self.__peek_service=rospy.Service(name+'/Peek',Peek,self.__peek)
        self.__pop_service=rospy.Service(name+'/Pop',Pop,self.__pop)


    def __del__(self):
        self.__exists_service.shutdown()
        self.__peek_service.shutdown()
        self.__pop_service.shutdown()

    def __converts_request_to_tuple_space_format(self,pattern):
        lst = list(pattern)
        for i in range (0,len(lst)):
            if (lst[i] == None):
                lst[i] = str
        return tuple(lst)

    def __push(self, request):
        # Since all read request converts nones to string type, it must be done also here.
        # Otherwise the stored tupple can't readed or removed anymore
        converted = self.__converts_request_to_tuple_space_format(request.content)
        self.__tuple_space.add(converted)

    def __exists(self, request):
        try:
            converted = self.__converts_request_to_tuple_space_format(request.pattern)
            self.__tuple_space.get(converted)
            return True
        except KeyError as e:
            return False

    def __peek(self, request):
        try:
            converted = self.__converts_request_to_tuple_space_format(request.pattern)
            result = self.__tuple_space.get(converted)
            return PeekResponse(example=result,exists=True)
        except KeyError as e:
            return PeekResponse(exists=False)

    def __pop(self, request):
        try:
            converted = self.__converts_request_to_tuple_space_format(request.pattern)
            result = self.__tuple_space.get(converted,remove=True)
            return PopResponse(removed=result,exists=True)
        except KeyError as e:
            return PopResponse(exists=False)