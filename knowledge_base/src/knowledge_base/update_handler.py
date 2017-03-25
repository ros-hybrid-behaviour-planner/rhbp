'''

@author: rieger
'''
import rospy
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.msg import FactRemoved, Fact, FactUpdated


class KnowledgeBaseFactCache:
    """
    Adapter for using update mechanism of knowledge base for caching the value.
    If the knowledge base does not exists at initialization, than the subscribe is done at first using of the cache.
    """

    def __init__(self, pattern, knowledge_base_name=KnowledgeBase.DEFAULT_NAME):

        self.__knowledge_base_name = knowledge_base_name
        self.__pattern = pattern
        self.__initialized = False
        self.__client = KnowledgeBaseClient(knowledge_base_name)
        self.__contained_facts = []
        self.__example_service_name = knowledge_base_name + KnowledgeBase.UPDATE_SERVICE_NAME_POSTFIX

        try:
            rospy.wait_for_service(self.__example_service_name, timeout=10)
            self.__register_for_updates()
        except rospy.ROSException:
            rospy.loginfo(
                'The following knowledge base node is currently not present. Connection will be established later: ' + knowledge_base_name)

    def __register_for_updates(self):
        """
        registers at knowledge base for updates of facts, which match the pattern of this instance
        """
        added_topic_name, update_topic_name, removed_topic_name = self.__client.subscribe_for_updates(self.__pattern)
        rospy.Subscriber(added_topic_name, Fact, self.__handle_add_update)
        rospy.Subscriber(removed_topic_name, FactRemoved, self.__handle_remove_update)
        rospy.Subscriber(update_topic_name, FactUpdated, self.__handle_fact_update)
        self.update_state_manually()
        self.__initialized = True
        rospy.logdebug('Connected to knowledge base: ' + self.__knowledge_base_name)

    def __handle_add_update(self, fact_added):
        """
        handles message, that a matching fact was added
        :param fact_added: empty message
        """
        if (not fact_added.content in self.__contained_facts):
            self.__contained_facts.append(tuple(fact_added.content))

    def __handle_remove_update(self, fact_removed):
        """
        handles message, that a matching fact was removed
        :param fact_removed: FactRemoved, as defined ROS message
        """
        try:
            self.__contained_facts.remove(tuple(fact_removed.fact))
        except KeyError:
            pass

    def __handle_fact_update(self, fact_updated):
        if (not fact_updated.new in self.__contained_facts):
            self.__contained_facts.append(tuple(fact_updated.new))

        for removed_fact in fact_updated.removed:
            try:
                self.__contained_facts.remove(tuple(removed_fact.content))
            except KeyError:
                pass

    def update_state_manually(self):
        """
        requests in knowledge base, whether a matching state exists
        :return: whether matching fact exists
        """
        self.__contained_facts = self.__client.all(self.__pattern)
        return not (len(self.__contained_facts) == 0)

    def __ensure_initialization(self):
        if not self.__initialized:
            rospy.loginfo('Wait for knowledge base service: ' + self.__example_service_name)
            rospy.wait_for_service(self.__example_service_name)
            self.__register_for_updates()

    def does_fact_exists(self):
        """
        :return: current cached value
        """
        self.__ensure_initialization()
        return not (len(self.__contained_facts) == 0)

    def get_all_matching_facts(self):
        self.__ensure_initialization()
        return self.__contained_facts
