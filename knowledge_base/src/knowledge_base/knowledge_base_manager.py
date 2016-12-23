#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: phillip
'''

import re
import sys

import rospy
from knowledge_base.msg import Push, Fact, FactRemoved
from knowledge_base.srv import Exists, Peek, PeekResponse, Pop, PopResponse, All, AllResponse, UpdateSubscribe, \
    UpdateSubscribeResponse
from lindypy.TupleSpace import TSpace
from std_msgs.msg import Empty

from inverted_tuple_space import InvertedTupleSpace

"""
Wrapper class for accessing the real tuple space
"""


class KnowledgeBase(object):
    DEFAULT_NAME = 'knowledgeBaseNode'
    EXISTS_SERVICE_NAME_POSTFIX = '/Exists'
    PEEK_SERVICE_NAME_POSTFIX = '/Peek'
    POP_SERVICE_NAME_POSTFIX = '/Pop'
    ALL_SERVICE_NAME_POSTFIX = '/All'
    UPDATE_SUBSCRIBER_NAME_POSTFIX = '/UpdateSubscriber'
    PUSH_TOPIC_NAME_POSTFIX = '/UpdateSubscriber'

    def __init__(self, name=DEFAULT_NAME, inlcude_patterns_in_update_names=False):
        self.__fact_update_topic_prefix = name + '/FactUpdate/'
        self.__fact_update_topic_counter = 0
        self.__inlcude_patterns_in_update_names = inlcude_patterns_in_update_names
        self.__tuple_space = TSpace()
        self.__subscribed_patterns_space = InvertedTupleSpace()
        self.__fact_update_topics = {}
        rospy.Subscriber(name + KnowledgeBase.PUSH_TOPIC_NAME_POSTFIX, Push, self.__push)
        self.__exists_service = rospy.Service(name + KnowledgeBase.EXISTS_SERVICE_NAME_POSTFIX, Exists, self.__exists)
        self.__peek_service = rospy.Service(name + KnowledgeBase.PEEK_SERVICE_NAME_POSTFIX, Peek, self.__peek)
        self.__pop_service = rospy.Service(name + KnowledgeBase.POP_SERVICE_NAME_POSTFIX, Pop, self.__pop)
        self.__all_service = rospy.Service(name + KnowledgeBase.ALL_SERVICE_NAME_POSTFIX, All, self.__all)
        self.__update_subscriber_service = rospy.Service(name + KnowledgeBase.UPDATE_SUBSCRIBER_NAME_POSTFIX, UpdateSubscribe,
                                                         self.__update_subscribe)

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
        replaces the placeholder string with string type
        :param pattern:  tuple of non-None strings
        :return: tuple. At each position in the source, where the placeholder * was, is now the typ str
        """
        lst = list(pattern)
        for i in range(0, len(lst)):
            if lst[i] == '*':
                lst[i] = str
        return tuple(lst)

    def __push(self, request):
        """
        Adds the fact to the tuple space
        :param request: Push message, as defined as ROS message
        """
        # Since all read request converts nones to string type, it must be done also here.
        # Otherwise the stored tuple can't readed or removed anymore
        converted = self.__converts_request_to_tuple_space_format(request.content)
        if not self.__exists_tuple_as_is(converted):
            self.__tuple_space.add(converted)
            self.__fact_was_added(converted)

    def __fact_was_added(self, fact):
        """
        informs all registered clients about change
        :param fact: tuple of strings
        """
        for pattern in self.__subscribed_patterns_space.find_for_fact(fact):
            add_update_topic = self.__fact_update_topics[pattern][0]
            add_update_topic.publish(Empty())

    def __exists_tuple_as_is(self, to_check):
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
        :return: bool, which indiciated, whether a tuple exists in knowledge base, which matchs the pattern
        """
        converted = self.__converts_request_to_tuple_space_format(exists_request.pattern)
        return self.__exists_tuple_as_is(converted)

    def __peek(self, peek_request):
        """
        :param request: Peek, as defined as ROS service
        :return: PeekResponse, as defined as ROS service respone
                if a tuple exists in tuple space, an example is returned and the exists flag is setted
        """
        try:
            converted = self.__converts_request_to_tuple_space_format(peek_request.pattern)
            result = self.__tuple_space.get(converted)
            return PeekResponse(example=result, exists=True)
        except KeyError:
            return PeekResponse(exists=False)

    def __pop(self, pop_request):
        """
        If tuple exists in the tuple space, which matchs the pattern, it will be also removed from  the tuple space
        and returned
        :param request: Pop, as defined as ROS service
        :return: PopResponse, as defined as ROS service respone
                if a tuple exists in tuple space, an example is returned and the exists flag is setted
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
        for pattern in self.__subscribed_patterns_space.find_for_fact(removed_fact):
            another_matching_fact_exists = self.__exists_tuple_as_is(pattern)
            removed_update_topic = self.__fact_update_topics[pattern][1]
            removed_update_topic.publish(
                FactRemoved(fact=removed_fact, another_matching_fact_exists=another_matching_fact_exists))

    def __all(self, all_request):
        """
        :return: all contained tuples, matching the given pattern, but never more tuples than sys.max_int
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
    def generate_topic_name_part_from_pattern(pattern):

        """
        Generate name for pattern which can be used in topic name for fact updates.
        Usefully for debugging
        :param pattern: array of strings or str types
        :return:
        """
        topic_name_part = ''
        first_part = True
        for part in pattern:
            if first_part:
                first_part = False
            else:
                topic_name_part += '_'

            if isinstance(part, type):
                topic_name_part += 'x'
            else:
                topic_name_part += re.sub(r'[^a-zA-Z0-9]', '', str(part))
        return topic_name_part

    @staticmethod
    def generate_topic_name_for_pattern(prefix, pattern, include_pattern, counter):
        """
        generates topic name for given pattern.
        Name collision is possible!
        :param prefix: prefix for topic names, class variable, is a parameter for allow unit tests
        :param pattern: converted pattern
        :return: topic name
        """
        topic_name = prefix + 'Topic' + str(counter)
        if include_pattern:
            topic_name += '_' + KnowledgeBase.generate_topic_name_part_from_pattern(pattern)
        return topic_name

    def __update_subscribe(self, update_subscribe_request):
        """
        register for updates of the given pattern
        :param update_subscribe_request: UpdateSubscribe, as defined ROS message
        """
        converted = self.__converts_request_to_tuple_space_format(update_subscribe_request.interested_pattern)
        if converted in self.__fact_update_topics:
            # Another client has already subscribed this pattern
            add_topic, remove_topic = self.__fact_update_topics[converted]
            return UpdateSubscribeResponse(add_topic_name=add_topic.name, remove_topic_name=remove_topic.name)

        basic_topic_name = KnowledgeBase.generate_topic_name_for_pattern(self.__fact_update_topic_prefix, converted,
                                                                         self.__inlcude_patterns_in_update_names,
                                                                         self.__fact_update_topic_counter)
        self.__fact_update_topic_counter += 1
        add_topic_name = basic_topic_name + '/Add'
        add_publisher = rospy.Publisher(add_topic_name, Empty, queue_size=10)
        remove_topic_name = basic_topic_name + '/Remove'
        remove_publisher = rospy.Publisher(remove_topic_name, FactRemoved, queue_size=10)
        rospy.sleep(1)
        self.__fact_update_topics[converted] = (add_publisher, remove_publisher)
        self.__subscribed_patterns_space.add(converted)
        return UpdateSubscribeResponse(remove_topic_name=remove_topic_name, add_topic_name=add_topic_name)
