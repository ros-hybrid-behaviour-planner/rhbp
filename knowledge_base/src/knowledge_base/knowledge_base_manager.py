#! /usr/bin/env python2
"""
Created on 07.12.2016

@author: rieger, hrabia
"""

import re
import sys
from threading import Lock, RLock

import rospy
from knowledge_base.msg import Fact, FactRemoved, FactUpdated
from knowledge_base.srv import Exists, Peek, PeekResponse, Pop, PopResponse, All, AllResponse, UpdateSubscribe, \
    UpdateSubscribeResponse, Update, UpdateResponse, Push, PushResponse

from inverted_tuple_space import InvertedTupleSpace

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.kb')


class KnowledgeBase(object):
    """
    Provides a tuple space for ROS. The tuple space is accessible through several ROS services:
    (Peek, Pop, Exists, All, Push).
    It allows subscribing for updates (see UpdateSubscribe service).
    Although this class is just a wrapper for accessing the real tuple space
    """

    DEFAULT_NAME = 'knowledgeBaseNode'
    EXISTS_SERVICE_NAME_POSTFIX = '/Exists'
    PEEK_SERVICE_NAME_POSTFIX = '/Peek'
    POP_SERVICE_NAME_POSTFIX = '/Pop'
    ALL_SERVICE_NAME_POSTFIX = '/All'
    UPDATE_SERVICE_NAME_POSTFIX = '/Update'
    UPDATE_SUBSCRIBE_SERVICE_NAME_POSTFIX = '/UpdateSubscriber'
    PUSH_SERVICE_NAME_POSTFIX = '/Push'
    DISCOVERY_TOPIC_NAME = '/kb_discover'

    PUBLISHER_IDX_ADD = 0
    PUBLISHER_IDX_REMOVE = 1
    PUBLISHER_IDX_UPDATE = 2

    PLACEHOLDER = '*'

    PLACEHOLDER_ALL = ()

    # This will let the publisher queue grow infinitely but if we would drop here clients might lose information
    KB_PUBLISHER_QUEUE_SIZE = 0

    def __init__(self, name=DEFAULT_NAME, include_patterns_in_update_names=False):
        self.__fact_update_topic_prefix = name + '/FactUpdate/'
        self.__fact_update_topic_counter = 0
        self.__include_patterns_in_update_names = include_patterns_in_update_names
        self.__tuple_space = InvertedTupleSpace()
        self.__subscribed_patterns_space = InvertedTupleSpace()
        self.__fact_update_topics = {}

        self.__register_lock = Lock()
        self.__tuple_lock = RLock()

        self.__push_service = rospy.Service(name + KnowledgeBase.PUSH_SERVICE_NAME_POSTFIX, Push, self.__push)
        self.__exists_service = rospy.Service(name + KnowledgeBase.EXISTS_SERVICE_NAME_POSTFIX, Exists, self.__exists)
        self.__peek_service = rospy.Service(name + KnowledgeBase.PEEK_SERVICE_NAME_POSTFIX, Peek, self.__peek)
        self.__pop_service = rospy.Service(name + KnowledgeBase.POP_SERVICE_NAME_POSTFIX, Pop, self.__pop)
        self.__all_service = rospy.Service(name + KnowledgeBase.ALL_SERVICE_NAME_POSTFIX, All, self.__all)
        self.__update_service = rospy.Service(name + KnowledgeBase.UPDATE_SERVICE_NAME_POSTFIX, Update, self.__update)
        self.__update_subscriber_service = rospy.Service(name + KnowledgeBase.UPDATE_SUBSCRIBE_SERVICE_NAME_POSTFIX,
                                                         UpdateSubscribe, self.__update_subscribe)
        
    def __del__(self):
        """
            closes all services
        """
        self.__exists_service.shutdown()
        self.__peek_service.shutdown()
        self.__pop_service.shutdown()
        self.__all_service.shutdown()
        self.__update_service.shutdown()
        self.__update_subscriber_service.shutdown()
        self.__push_service.shutdown()

    @staticmethod
    def __converts_request_to_tuple_space_format(pattern):
        """
        replaces the placeholder string with string type
        :param pattern:  tuple of non-None strings
        :return: tuple. At each position in the source, where the placeholder * was, is now the typ str
        """
        lst = list(pattern)
        for i in range(0, len(lst)):
            if lst[i] == KnowledgeBase.PLACEHOLDER:
                lst[i] = str
        return tuple(lst)

    def __push(self, request):
        """
        Adds the fact to the tuple space
        :param request: Push message, as defined as ROS service
        :return: an empty PushResponse
        """
        # Since all read request converts nones to string type, it must be done also here.
        # Otherwise the stored tuple can't read or removed anymore
        converted = self.__converts_request_to_tuple_space_format(request.content)
        rhbplog.logdebug('New tuple {0}'.format(str(request.content)))
        self.__push_internal(request.content, converted, None)
        return PushResponse(successful=True)

    def __push_internal(self, unconverted_fact, fact, dont_inform=None):
        """
        difference to __push: fact and don't_inform are already converted
        all patterns, which matches the don't_inform fact, will not informed about push
        """
        for part in unconverted_fact:
            # Placeholders are not allowed in facts
            assert not part == str
        if not self.__exists_tuple_as_is(fact):
            try:
                self.__tuple_lock.acquire()
                self.__tuple_space.add(fact)
            finally:
                self.__tuple_lock.release()
            self.__fact_was_added(Fact(unconverted_fact), fact, dont_inform)

    def __fact_was_added(self, sendeable_fact, original_fact, dont_inform):
        """
        informs all registered clients about change
        :param sendeable_fact: ros message fact
        :param original_fact: tupple of strings
        :param fact: tuple of strings
        :param dont_inform: pattern that should not be notified about changes, None to notify all
        """
        if (dont_inform is None):
            excluded = []
        else:
            excluded = self.__subscribed_patterns_space.find_for_fact(dont_inform)
        for pattern in self.__subscribed_patterns_space.find_for_fact(original_fact):
            if (pattern in excluded):
                continue
            add_update_topic = self.__fact_update_topics[pattern][KnowledgeBase.PUBLISHER_IDX_ADD]
            add_update_topic.publish(sendeable_fact)
        # check for the empty tuple aka all placeholder
        # only run this without dont_inform, means it is a direct insert
        if dont_inform is None and self.__has_all_updates_subscriber():
            add_update_topic = self.__fact_update_topics[KnowledgeBase.PLACEHOLDER_ALL][KnowledgeBase.PUBLISHER_IDX_ADD]
            add_update_topic.publish(sendeable_fact)

    def __has_all_updates_subscriber(self):
        """
        Checks if there is a subscriber for all updates, using a empty tuple pattern
        :return: True for such subscriber available
        """
        return len(self.__subscribed_patterns_space.many(pattern=KnowledgeBase.PLACEHOLDER_ALL, number=1)) > 0

    def __exists_tuple_as_is(self, to_check):
        """
        Checks whether the tuple is returned in tuple space. Just a wrapper method.
        Does no conversion of the requested tuple
        :return: whether the requested tuple is contained in the tuple space.
        """
        try:
            self.__tuple_space.get(to_check)
        except KeyError:
            return False
        return True

    def __exists(self, exists_request):
        """
        :param request: Exists, as defined as ROS service
        :return: bool, which indiciated, whether a tuple exists in knowledge base, which matchs the pattern
        """
        with self.__tuple_lock:
            converted = self.__converts_request_to_tuple_space_format(exists_request.pattern)
        return self.__exists_tuple_as_is(converted)

    def __peek(self, peek_request):
        """
        :param request: Peek, as defined as ROS service
        :return: PeekResponse, as defined as ROS service respone
                if a tuple exists in tuple space, an example is returned and the exists flag is setted
        """
        try:
            self.__tuple_lock.acquire()
            converted = self.__converts_request_to_tuple_space_format(peek_request.pattern)
            result = self.__tuple_space.get(converted)
            return PeekResponse(example=result, exists=True)
        except KeyError:
            return PeekResponse(exists=False)
        finally:
            self.__tuple_lock.release()

    def __pop_internal(self, pattern, dont_inform=None):
        """
        :param pattern: fact, which should be removed
        :param dont_inform: all subscribers, which matches this fact are not informed
        """

        with self.__tuple_lock:
            to_remove = self.__tuple_space.many(pattern, sys.maxint)
            removed_facts = []
            for fact in to_remove:
                real_fact = fact[1]
                try:
                    self.__tuple_space.get(real_fact, remove=True)
                    removed_facts.append(Fact(content=real_fact))
                    self.__fact_removed(real_fact, dont_inform)
                except KeyError:
                    pass
            return PopResponse(removed=tuple(removed_facts))

    def __pop(self, pop_request):
        """
        All matching tuples in the tuple space will be removed.
        :param request: Pop, as defined as ROS service
        :return: PopResponse, all removed tuples
        """
        converted = self.__converts_request_to_tuple_space_format(pop_request.pattern)
        return self.__pop_internal(converted)

    def __fact_removed(self, removed_fact, dont_inform):
        """
        informs all registered clients about remove of the fact
        :param removed_fact: tuple of strings
        """
        if (dont_inform is None):
            exluded = []
        else:
            exluded = self.__subscribed_patterns_space.find_for_fact(dont_inform)

        another_matching_fact_exists = False
        for pattern in self.__subscribed_patterns_space.find_for_fact(removed_fact):
            if pattern in exluded:
                continue
            another_matching_fact_exists = self.__exists_tuple_as_is(pattern)
            removed_update_topic = self.__fact_update_topics[pattern][KnowledgeBase.PUBLISHER_IDX_REMOVE]
            removed_update_topic.publish(
                FactRemoved(fact=removed_fact, another_matching_fact_exists=another_matching_fact_exists))
        # handle the all placeholder
        # only run this without dont_inform (means it is a direct insert
        if dont_inform is None and self.__has_all_updates_subscriber():
            removed_update_topic = self.__fact_update_topics[KnowledgeBase.PLACEHOLDER_ALL][KnowledgeBase.PUBLISHER_IDX_REMOVE]
            removed_update_topic.publish(
                FactRemoved(fact=removed_fact, another_matching_fact_exists=another_matching_fact_exists))

    def __all(self, all_request):
        """
        :return: all contained tuples, matching the given pattern, but never more tuples than sys.max_int
        """
        converted = self.__converts_request_to_tuple_space_format(all_request.pattern)
        try:
            self.__tuple_lock.acquire()
            if len(converted) == 0:
                found_tuples = self.__tuple_space.all()
            else:
                found_tuples = self.__tuple_space.many(converted, sys.maxint)
            result_as_list = []
            for fact in found_tuples:
                result_as_list.append(Fact(content=fact[1]))
            return AllResponse(found=tuple(result_as_list))
        except KeyError:
            return ()
        finally:
            self.__tuple_lock.release()

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

    def __update_subscribe_not_thread_safe(self, update_subscribe_request):
        """
        see __update_subscribe.
        Ensure, that NEVER several threads are at the same time in this method
        :param update_subscribe_request:
        :return:
        """
        converted = self.__converts_request_to_tuple_space_format(update_subscribe_request.interested_pattern)
        if converted in self.__fact_update_topics:
            # Another client has already subscribed this pattern
            added_topic, removed_topic, update_topic = self.__fact_update_topics[converted]
            return UpdateSubscribeResponse(added_topic_name=added_topic.name, removed_topic_name=removed_topic.name,
                                           updated_topic_name=update_topic.name)

        basic_topic_name = KnowledgeBase.generate_topic_name_for_pattern(self.__fact_update_topic_prefix, converted,
                                                                         self.__include_patterns_in_update_names,
                                                                         self.__fact_update_topic_counter)
        self.__fact_update_topic_counter += 1
        added_topic_name = basic_topic_name + '/Add'
        add_publisher = rospy.Publisher(added_topic_name, Fact, queue_size=self.KB_PUBLISHER_QUEUE_SIZE, latch=True)
        removed_topic_name = basic_topic_name + '/Remove'
        remove_publisher = rospy.Publisher(removed_topic_name, FactRemoved, queue_size=self.KB_PUBLISHER_QUEUE_SIZE,
                                           latch=True)
        update_topic_name = basic_topic_name + '/Update'
        update_publisher = rospy.Publisher(update_topic_name, FactUpdated, queue_size=self.KB_PUBLISHER_QUEUE_SIZE,
                                           latch=True)
        rospy.sleep(0.1)
        self.__fact_update_topics[converted] = (add_publisher, remove_publisher, update_publisher)
        self.__subscribed_patterns_space.add(converted)
        return UpdateSubscribeResponse(removed_topic_name=removed_topic_name, added_topic_name=added_topic_name,
                                       updated_topic_name=update_topic_name)

    def __update_subscribe(self, update_subscribe_request):
        """
        register for updates of the given pattern
        :param update_subscribe_request: UpdateSubscribe, as defined ROS message
        """
        self.__register_lock.acquire()
        try:
            return self.__update_subscribe_not_thread_safe(update_subscribe_request)
        finally:
            self.__register_lock.release()

    def __fact_updated(self, removed_facts, new_fact, converted_new_fact):
        """
        informs all clients, which are interested about added and at least one removed about update
        :param removed_facts: network compatible version of all removed facts, encapsulated in ROS message class Fact
        :param new_fact: network compatible version of fact
        """

        interested_in_new = self.__subscribed_patterns_space.find_for_fact(converted_new_fact)
        facts_by_interested_pattern = {}

        # Collect all facts for each pattern to send just one update message per client
        for fact in removed_facts:
            interested_in_old = self.__subscribed_patterns_space.find_for_fact(fact.content)
            to_inform = set(interested_in_old).intersection(interested_in_new)
            for pattern in to_inform:
                if pattern not in facts_by_interested_pattern:
                    facts_by_interested_pattern[pattern] = []
                facts_by_interested_pattern[pattern].append(fact.content)

        for pattern in facts_by_interested_pattern.keys():
            facts = []
            for fact in facts_by_interested_pattern[pattern]:
                facts.append(Fact(fact))
            update_topic = self.__fact_update_topics[pattern][KnowledgeBase.PUBLISHER_IDX_UPDATE]
            update_topic.publish(FactUpdated(removed=tuple(facts), new=new_fact))
        # handle the all placeholder
        if self.__has_all_updates_subscriber():
            update_topic = self.__fact_update_topics[KnowledgeBase.PLACEHOLDER_ALL][KnowledgeBase.PUBLISHER_IDX_UPDATE]
            update_topic.publish(FactUpdated(removed=removed_facts, new=new_fact))

    def __update(self, update_request):
        """
        :param update_request: ROS service request Update
        :return: true if replacing was successful, false if not because old fact does not exists
        """
        pattern = self.__converts_request_to_tuple_space_format(update_request.pattern)
        new_fact = self.__converts_request_to_tuple_space_format(update_request.newFact)

        with self.__tuple_lock: # it is a RLock for this reason it is safe to acquire it here as well as in push and pop

            if self.__exists_tuple_as_is(new_fact):
                self.__pop_internal(new_fact, dont_inform=new_fact)
                was_existing_before = True
            else:
                was_existing_before = False

            removed_facts = self.__pop_internal(pattern, dont_inform=new_fact).removed
            facts_removed = len(removed_facts) > 0
            if not update_request.pushWithoutExisting and not facts_removed:
                # If no facts was removed (no update), no clients will be informed and the fact will not be stored
                return UpdateResponse(False)

            if facts_removed:
                self.__push_internal(update_request.newFact, new_fact, dont_inform=pattern)
            elif was_existing_before:
                self.__push_internal(update_request.newFact, new_fact, dont_inform=new_fact)
            else:
                self.__push_internal(update_request.newFact, new_fact, dont_inform=None)

        self.__fact_updated(removed_facts, update_request.newFact, new_fact)
        return UpdateResponse(True)
