'''

@author: phillip
'''
import rospy
from knowledge_base.msg import FactAdded, FactRemoved
from knowledge_base.srv import UpdateSubscribe, Exists


class KnowledgeBaseFactCache:
    """
    Adapter for using update mechanism of knowledge base for caching the value.
    If the value does not exists at initialization, than the subscribe is done at first using
    """

    def __init__(self, pattern, knowledge_base_name=None):

        if knowledge_base_name is None:
            # allow directly forwarding a parameter at constructor call
            knowledge_base_name = 'knowledgeBaseNode'

        self.__pattern = pattern
        self.__value = False
        self.__initialized = False
        self.__knowledge_base_update_subscriber_service_name = knowledge_base_name + '/UpdateSubscriber'
        self.__exists_service_name = knowledge_base_name + '/Exists'
        try:
            rospy.wait_for_service(self.__knowledge_base_update_subscriber_service_name, timeout=10)
        except rospy.ROSException:
            pass

    def __register_for_updates(self):
        """
        registers at knowledge base for updates of facts, which match the pattern of this instance
        """
        self.__initialized = True
        register_for_updates_services = rospy.ServiceProxy(self.__knowledge_base_update_subscriber_service_name,
                                                           UpdateSubscribe)
        response = register_for_updates_services(self.__pattern)
        rospy.Subscriber(response.add_topic_name, FactAdded, self.__handle_add_update)
        rospy.Subscriber(response.remove_topic_name, FactRemoved, self.__handle_remove_update)
        self.update_state_manually()

    def __match_pattern(self, fact):
        """
        :param fact: tupple of strings
        :return: whether given fact matches the pattern of this cache
        """
        if not (len(fact) == len(self.__pattern)):
            return False
        for index, pattern_part in enumerate(self.__pattern):
            if not (pattern_part == '*' or pattern_part == fact[index]):
                return False
        return True

    def __handle_add_update(self, fact_added):
        """
        handles message, that a matching fact was added
        :param factAdded: FactAdded, as defined ROS message
        """
        if self.__match_pattern(fact_added.fact):
            self.__value = True

    def __handle_remove_update(self, fact_removed):
        """
        handles message, that a matching fact was removed
        :param fact_removed: FactRemoved, as defined ROS message
        """
        if self.__match_pattern(fact_removed.fact):
            self.__value = fact_removed.another_matching_fact_exists

    def update_state_manually(self):
        """
        requests in knowledge base, whether a matching state exists
        :return: whether matching fact exists
        """
        rospy.wait_for_service(self.__exists_service_name)
        exists_service = rospy.ServiceProxy(self.__exists_service_name, Exists)
        self.__value = exists_service(self.__pattern).exists
        return self.__value

    def does_fact_exists(self):
        """
        :return: current cached value
        """
        if not self.__initialized:
            rospy.wait_for_service(self.__knowledge_base_update_subscriber_service_name)
            self.__register_for_updates()
        return self.__value
