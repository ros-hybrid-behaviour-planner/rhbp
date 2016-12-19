#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: phillip
'''

import sys

import rospy
from knowledge_base.msg import Push, Fact, FactAdded, FactRemoved
from knowledge_base.srv import Exists, Peek, PeekResponse, Pop, PopResponse, All, AllResponse, UpdateSubscribe, \
    UpdateSubscribeResponse
from lindypy.TupleSpace import TSpace

from inverted_tuple_space import InvertedTupleSpace

"""
Wrapper class for accessing the real tupple space
"""


class KnowledgeBase(object):
    def __init__(self, name):
        self.__update_topic_prefix = name + '/FactUpdate/'
        self.__tuple_space = TSpace()
        self.__subscribed_patterns_space = InvertedTupleSpace()
        self.__fact_update_topics = {}
        rospy.Subscriber(name + '/Push', Push, self.__push)
        self.__exists_service = rospy.Service(name + '/Exists', Exists, self.__exists)
        self.__peek_service = rospy.Service(name + '/Peek', Peek, self.__peek)
        self.__pop_service = rospy.Service(name + '/Pop', Pop, self.__pop)
        self.__all_service = rospy.Service(name + '/All', All, self.__all)
        self.__update_subscriber_service = rospy.Service(name + '/UpdateSubscriber', UpdateSubscribe,
                                                         self.__update_subscribe())

    def __del__(self):
        """
            closes all services
        """
        self.__exists_service.shutdown()
        self.__peek_service.shutdown()
        self.__pop_service.shutdown()
        self.__all_service.shutdown()
        self.__update_subscriber_service.shutdown()

    @staticmethod
    def __converts_request_to_tuple_space_format(pattern):
        """
        :param pattern:  tupple of non-None strings
        :return: tupple. At each position in the source, where the placeholder * was, is now the typ str
        """
        lst = list(pattern)
        for i in range(0, len(lst)):
            if lst[i] == '*':
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
            self.__fact_was_added()

    def __fact_was_added(self,fact):
        """
        informs all registered clients about change
        :param fact: tupple of strings
        """
        for pattern in self.__subscribed_patterns_space:
            add_update_topic =  self.__fact_update_topics[pattern][0]
            add_update_topic.pulish(FactAdded(fact))

    def __exists_tupple_as_is(self, to_check):
        """
        Checks whether the tuple is returned in tuple space. Just a wrapper method.
        Does no conversion of the requested tuple
        :return: whether the requested tuple is contained in the tuple space.
        """
        try:
            self.__tuple_space.get(to_check)
            return True
        except KeyError:
            return False

    def __exists(self, exists_request):
        """
        :param request: Exists, as defined as ROS service
        :return: bool, which indiciated, whether a tupple exists in knowledge base, which matchs the pattern
        """
        converted = self.__converts_request_to_tuple_space_format(exists_request.pattern)
        return self.__exists_tupple_as_is(converted)

    def __peek(self, peek_request):
        """
        :param request: Peek, as defined as ROS service
        :return: PeekResponse, as defined as ROS service respone
                if a tupple exists in tupple space, an example is returned and the exists flag is setted
        """
        try:
            converted = self.__converts_request_to_tuple_space_format(peek_request.pattern)
            result = self.__tuple_space.get(converted)
            return PeekResponse(example=result, exists=True)
        except KeyError:
            return PeekResponse(exists=False)

    def __pop(self, pop_request):
        """
        If tupple exists in the tupple space, which matchs the pattern, it will be also removed from  the tupple space
        and returned
        :param request: Pop, as defined as ROS service
        :return: PopResponse, as defined as ROS service respone
                if a tupple exists in tupple space, an example is returned and the exists flag is setted
        """
        converted = self.__converts_request_to_tuple_space_format(pop_request.pattern)
        try:
            result = self.__tuple_space.get(converted, remove=True)
            self.__fact_removed(result)
            return PopResponse(removed=result, exists=True)
        except KeyError:
            return PopResponse(exists=False)

    def __fact_removed(self, removed_fact):
        """
        informs all registered clients about remove of the fact
        :param removed_fact: tuple of strings
        """
        for pattern in self.__subscribed_patterns_space:
            another_matching_fact_exists = self.__exists_tupple_as_is(pattern)
            removed_update_topic =  self.__fact_update_topics[pattern][0]
            removed_update_topic.pulish(FactRemoved(fact=removed_fact,another_matching_fact_exists = another_matching_fact_exists))

    def __all(self, all_request):
        """
        :return: all contained tupples, matching the given pattern, but never more tuples than sys.max_int
        """
        converted = self.__converts_request_to_tuple_space_format(all_request.pattern)
        try:
            found_tuples = self.__tuple_space.many(converted, sys.maxint)
            result_as_list = []
            for fact in found_tuples:
                result_as_list.append(Fact(content=fact[1]))
            return AllResponse(found=tuple(result_as_list))
        except KeyError:
            return ()

    @staticmethod
    def generate_topic_name_for_pattern(prefix, pattern):
        """

        :param prefix: prefix for topic names, class variable, is a parameter for allow unit tests
        :param pattern: converted pattern
        :return: topic name
        """
        topic_name = prefix
        first_part = True
        for part in pattern:
            if first_part:
                first_part = False
            else:
                topic_name += '_'

            if isinstance(part, type):
                topic_name += '~'
            else:
                topic_name += part
        return topic_name

    def __update_subscribe(self, update_subscribe_request):
        """
        register for updates of the given pattern
        :param update_subscribe_request: UpdateSubscribe, as defined ROS message
        """
        converted = self.__converts_request_to_tuple_space_format(update_subscribe_request.interested_pattern)
        if converted in self.__fact_update_topics:
            # Another client has already subscribed this pattern
            add_topic, remove_topic = self.__fact_update_topic[converted]
            return UpdateSubscribeResponse(add_topic=add_topic.name, remove_topic=remove_topic.name)

        basic_topic_name = KnowledgeBase.generate_topic_name_for_pattern(self.__update_topic_prefix, converted)
        add_topic_name = basic_topic_name + '/Add'
        addPublisher = rospy.Publisher(add_topic_name, FactAdded)
        remove_topic_name = basic_topic_name + '/Remove'
        removedPublisher = rospy.Publisher(remove_topic_name, FactRemoved)
        rospy.sleep(1)
        self.__fact_update_topics[converted] = (addPublisher, removedPublisher)
        self.__subscribed_patterns_space.add(converted)
