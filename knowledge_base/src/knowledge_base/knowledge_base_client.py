"""
Created on 09.03.2017

@author: rieger
"""

from threading import Lock

import rospy
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.srv import Exists, Peek, Pop, All, Update, UpdateSubscribe, Push
from thread import allocate_lock

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.kb')


class KnowledgeBaseClient(object):
    """
    Simple wrapper for the services (and the push topic) of the knowledge base.
    Some methods do some value conversion, for better usability.
    IMPORTANT: The service proxies are created once, but the initialization is lazy:
    The init method waits for the knowledge base for the duration of the timeout parameter.
    After the timeout the constructor returns and the initialisation is done during first usage of this client
    (without timeout).
    Patterns can contain a placeholder of "*" that matches to any entry, however pattern access does evaluate the pattern
    length. This means that a pattern of ("*", "*") does only compare/match to tuples with two fields
    An empty pattern can be used to retain all tuples
    """

    def __init__(self, knowledge_base_name=KnowledgeBase.DEFAULT_NAME, timeout=2):
        """
        :param knowledge_base_name: Name of the knowledge base (without any postfix)
        :param timeout: Timeout for the knowledge base service discovery
        :type timeout: int in seconds
        """
        self.__initialized = False
        self.__knowledge_base_name = knowledge_base_name
        self.__init_lock = allocate_lock()
        self._timeout = timeout

        try:
            rospy.wait_for_service(knowledge_base_name + KnowledgeBase.EXISTS_SERVICE_NAME_POSTFIX, timeout=self._timeout)
            self.__initialize()
        except rospy.ROSException:
            rhbplog.loginfo(
                'The following knowledge base node is currently not present. Connection will be established later: ' + knowledge_base_name)

    def __ensure_initialization(self):

        if self.__initialized:
            return True

        self.__init_lock.acquire()

        try:
            if self.__initialized:
                # Another check, protected by the lock
                return True
            rhbplog.logdebug(
                'Wait for knowledge base: ' + self.__knowledge_base_name + KnowledgeBase.EXISTS_SERVICE_NAME_POSTFIX)
            rospy.wait_for_service(self.__knowledge_base_name + KnowledgeBase.EXISTS_SERVICE_NAME_POSTFIX, timeout=self._timeout)
            self.__initialize()
            return True
        except rospy.ROSException:
            rhbplog.logwarn(
                'The following knowledge base node is currently not present. Connection will be established later: ' + self.__knowledge_base_name)
            return False
        finally:
            self.__init_lock.release()

    def __initialize(self):
        """
        initialize the client. Assumes, that the knowledge base already exists.
        Not thread safe
        """
        self.__exists_service = rospy.ServiceProxy(
            self.__knowledge_base_name + KnowledgeBase.EXISTS_SERVICE_NAME_POSTFIX, Exists)
        self.__pop_service = rospy.ServiceProxy(self.__knowledge_base_name + KnowledgeBase.POP_SERVICE_NAME_POSTFIX,
                                                Pop)
        self.__peek_service = rospy.ServiceProxy(self.__knowledge_base_name + KnowledgeBase.PEEK_SERVICE_NAME_POSTFIX,
                                                 Peek)
        self.__all_service = rospy.ServiceProxy(self.__knowledge_base_name + KnowledgeBase.ALL_SERVICE_NAME_POSTFIX,
                                                All)
        self.__update_service = rospy.ServiceProxy(
            self.__knowledge_base_name + KnowledgeBase.UPDATE_SERVICE_NAME_POSTFIX, Update)
        self.__update_subscribe_service = rospy.ServiceProxy(
            self.__knowledge_base_name + KnowledgeBase.UPDATE_SUBSCRIBE_SERVICE_NAME_POSTFIX, UpdateSubscribe)
        self.__push_service = rospy.ServiceProxy(self.__knowledge_base_name + KnowledgeBase.PUSH_SERVICE_NAME_POSTFIX,
                                                 Push)
        self.__initialized = True

    def exists(self, pattern):
        """
        Whether a fact, which matches the given pattern exists in the knowledge base
        :param pattern: (array of strings) pattern, use * as placeholder
        :return: (bool) result
        """
        if self.__ensure_initialization():
            return self.__exists_service(pattern).exists

    def pop(self, pattern):
        """
        Removes a fact, which matches the pattern from the knowledge base. If not suitable fact exists, nothing happens
        :param pattern:  (array or tuple  of strings) pattern, use * as placeholder
        :return: All removed facts as list of list of strings
        """
        if self.__ensure_initialization():
            request_result = self.__pop_service(pattern)
            removed = []
            for fact in request_result.removed:
                removed.append(tuple(fact.content))
            return removed
        else:
            return None

    def peek(self, pattern):
        """
        :param pattern:  (array or tuple  of strings) pattern, use * as placeholder
        :return: matching fact (string tupple) or none if not matching fact exists
        """
        if self.__ensure_initialization():
            request_result = self.__peek_service(pattern)
            if (request_result.exists):
                return tuple(request_result.example)
        return None

    def all(self, pattern):
        """
        :param pattern:  (array or tuple of strings) pattern, use * as placeholder
        :return: all matching facts, as list of string tuples
        """
        if self.__ensure_initialization():
            request_result = self.__all_service(pattern)
            result = []
            for fact in request_result.found:
                result.append(tuple(fact.content))
            return result
        else:
            return None

    def update(self, pattern, new, push_without_existing=True):
        """
        :param old:  fact, which should replaced.
        :param new: new fact
        :param push_without_existing: True for creating a new fact if it does not yet exist
        :return: whether new fact exists now in the knowledge base
        """
        if self.__ensure_initialization():
            return self.__update_service(pattern, new, push_without_existing).successful
        else:
            return False

    def push(self, fact):
        """
        WARNING: This operation is executed asynchronus
        :param fact: array or tuple  of strings. No placeholders are allowed
        """
        if self.__ensure_initialization():
            return self.__push_service(fact).successful
        else:
            return None

    def subscribe_for_updates(self, pattern):
        """
        :param pattern:  (array or tuple of strings) pattern, use * as placeholder
        :return: added topic name, updated topic name, removed topic name
        """
        if self.__ensure_initialization():
            request_result = self.__update_subscribe_service(pattern)
            return request_result.added_topic_name, request_result.updated_topic_name, request_result.removed_topic_name
        else:
            return None
