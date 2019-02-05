'''

@author: rieger
'''
import rospy

import threading
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.msg import FactRemoved, Fact, FactUpdated

import copy

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.kb')


class KnowledgeBaseFactCache(object):
    """
    Adapter for using update mechanism of knowledge base for caching the value.
    If the knowledge base does not exists at initialization, than the subscribe is done at first using of the cache.
    """

    def __init__(self, pattern, knowledge_base_name=KnowledgeBase.DEFAULT_NAME, timeout=None):
        """
        :param pattern: tuple of strings. The update handler caches all facts, matching the pattern.
        :param knowledge_base_name:
        :param timeout: Timeout for all kb service calls in seconds, default is blocking/endless
        """

        self.__knowledge_base_name = knowledge_base_name
        self.__service_timeout = timeout
        self.__pattern = pattern
        self.__initialized = False
        self.__contained_facts = []
        self.__example_service_name = knowledge_base_name + KnowledgeBase.UPDATE_SERVICE_NAME_POSTFIX
        self.__client = KnowledgeBaseClient(knowledge_base_name)
        self.__value_lock = threading.Lock()
        self.__update_listeners = []  # functions to call on update
        self.__update_time = None
        self.__last_updated_fact = tuple()

        try:
            initial_timeout = timeout if timeout else 5 # use here also provided timeout for initial non crucial waiting
            rospy.wait_for_service(self.__example_service_name, initial_timeout)
            self.__register_for_updates()
        except rospy.ROSException:
            rhbplog.logwarn(
                'The following knowledge base node is currently not present. Connection will be established later: '
                + knowledge_base_name)

    def __register_for_updates(self):
        """
        registers at knowledge base for updates of facts, which match the pattern of this instance
        """
        added_topic_name, update_topic_name, removed_topic_name = self.__client.subscribe_for_updates(self.__pattern)
        rospy.Subscriber(added_topic_name, Fact, self.__handle_add_update)
        rospy.Subscriber(removed_topic_name, FactRemoved, self.__handle_remove_update)
        rospy.Subscriber(update_topic_name, FactUpdated, self.__handle_fact_update)
        self.update_state_manually()
        rospy.sleep(0.1)  # Short sleep guarantees that we do not miss updates (triggering publisher queue processing)
        self.__initialized = True
        rhbplog.logdebug('Connected to knowledge base: ' + self.__knowledge_base_name)

    def __handle_add_update(self, fact_added):
        """
        handles message, that a matching fact was added
        :param fact_added: empty message
        """
        tuple_fact = tuple(fact_added.content)
        with self.__value_lock:
            if tuple_fact not in self.__contained_facts:
                self.__contained_facts.append(tuple_fact)
                self._cache_updated(tuple_fact)
        self._notify_listeners()

    def __handle_remove_update(self, fact_removed):
        """
        handles message, that a matching fact was removed
        :param fact_removed: FactRemoved, as defined ROS message
        """
        tuple_fact = tuple(fact_removed.fact)
        with self.__value_lock:
            try:
                self.__contained_facts.remove(tuple_fact)
            except ValueError:
                pass
        self._cache_updated(tuple_fact)
        self._notify_listeners()

    def __handle_fact_update(self, fact_updated):
        tuple_fact_new = tuple(fact_updated.new)
        with self.__value_lock:

            if tuple_fact_new not in self.__contained_facts:
                self.__contained_facts.append(tuple_fact_new)

            for removed_fact in fact_updated.removed:
                tuple_fact_old = tuple(removed_fact.content)
                if tuple_fact_old != tuple_fact_new:
                    try:
                        self.__contained_facts.remove(tuple_fact_old)
                    except ValueError:
                        pass
        self._cache_updated(tuple_fact_new)
        self._notify_listeners()

    def update_state_manually(self, enable_listener_notification=True):
        """
        requests/polls from knowledge base, whether a matching state exists
        :return: whether matching fact exists
        """
        new_content = self.__client.all(self.__pattern)
        with self.__value_lock:
            self.__contained_facts = new_content
            last_fact = self.__contained_facts[-1] if len(self.__contained_facts) > 0 else tuple()
            self._cache_updated(last_fact)  # last element of all facts
            if enable_listener_notification:
                self._notify_listeners()
            return not (len(self.__contained_facts) == 0)

    def __ensure_initialization(self):
        if not self.__initialized:
            try:
                rhbplog.loginfo('Wait for knowledge base service: ' + self.__example_service_name)
                rospy.wait_for_service(self.__example_service_name, timeout=self.__service_timeout)
                self.__register_for_updates()
            except rospy.ROSException as e:
                raise rospy.ROSException("Timeout: Cannot reach service self.__example_service_name:" + str(e))

    def does_fact_exists(self):
        """
        :return: current cached value
        """
        self.__ensure_initialization()
        with self.__value_lock:
            return not (len(self.__contained_facts) == 0)

    def does_sub_fact_exist(self, pattern):
        """
        Returns whether a fact contained in this KnowledgeBaseFactCache and additionally matching the
        given pattern exists.
        :param pattern: the pattern to look for
        :return: True if a cached fact matches the given pattern, else False
        """
        self.__ensure_initialization()
        with self.__value_lock:
            for f in self.__contained_facts:
                for i, pattern_elem in enumerate(pattern):
                    if pattern_elem != KnowledgeBase.PLACEHOLDER and not pattern_elem == f[i]:
                        break
                else:
                    return True
        return False

    def get_all_matching_facts(self):
        self.__ensure_initialization()
        with self.__value_lock:
            return copy.deepcopy(self.__contained_facts)

    def get_matching_sub_fact(self, pattern):
        """
        Returns the first fact that matches the given pattern, which should be a subset (i.e. more specific)
        of the pattern used during initialization of this KnowledgeBaseFactCache.
        :param pattern: the pattern to look for
        :return: the first contained fact matching the given pattern
        """
        self.__ensure_initialization()
        with self.__value_lock:
            for f in self.__contained_facts:
                for i, pattern_elem in enumerate(pattern):
                    if pattern_elem != KnowledgeBase.PLACEHOLDER and not pattern_elem == f[i]:
                        break
                else:
                    return copy.deepcopy(f)

    def get_all_matching_sub_facts(self, pattern):
        """
        Returns all facts that match the given pattern, which should be a subset (i.e. more specific) of the pattern
        used during initialization of this KnowledgeBaseFactCache.
        :param pattern: the pattern that facts should match
        :return: a list of contained facts matching the pattern
        """
        self.__ensure_initialization()
        with self.__value_lock:
            result = []
            for f in self.__contained_facts:
                for i, pattern_elem in enumerate(pattern):
                    if pattern_elem != KnowledgeBase.PLACEHOLDER and not pattern_elem == f[i]:
                        break
                else:
                    result.append(copy.deepcopy(f))
            return result

    def _notify_listeners(self):
        """
        Notify listeners about new fact update
        """
        for func in self.__update_listeners:
            func()

    def add_update_listener(self, func):
        """
        Add a function that is going to be called/notified on a fact update
        :param func: any python function without required parameters
        """
        if func not in self.__update_listeners:
            self.__update_listeners.append(func)

    def remove_update_listener(self, func):
        """
        Remove a function from the listener list
        :param func: function that has been registered before
        """
        if func in self.__update_listeners:
            self.__update_listeners.remove(func)

    def _cache_updated(self, fact):
        """
        function should be triggered when the cache was updated
        it stores some information about the last update
        :param fact: last updated fact
        """
        self.__update_time = rospy.Time.now()
        self.__last_updated_fact = fact

    @property
    def update_time(self):
        """
        Get time of last update
        :return: ROSTime of last fact change
        """
        return self.__update_time

    @property
    def last_updated_fact(self):
        """
        Get last updated fact
        :return: fact
        """
        return self.__last_updated_fact
