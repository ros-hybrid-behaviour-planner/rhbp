'''

@author: rieger
'''
import rospy
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.msg import FactRemoved
from knowledge_base.srv import UpdateSubscribe, All
from std_msgs.msg import Empty


class KnowledgeBaseFactCache:
    """
    Adapter for using update mechanism of knowledge base for caching the value.
    If the value does not exists at initialization, than the subscribe is done at first using
    """

    def __init__(self, pattern, knowledge_base_name=None):

        if knowledge_base_name is None:
            # allow directly forwarding a parameter at constructor call
            knowledge_base_name = KnowledgeBase.DEFAULT_NAME

        self.__pattern = pattern
        self.__initialized = False
        self.__knowledge_base_update_subscriber_service_name = knowledge_base_name + KnowledgeBase.UPDATE_SUBSCRIBER_NAME_POSTFIX
        self.__all_service_name = knowledge_base_name + KnowledgeBase.ALL_SERVICE_NAME_POSTFIX
        self.__knowledge_base_name = knowledge_base_name
        self.__contained_facts = []

        try:
            rospy.wait_for_service(self.__knowledge_base_update_subscriber_service_name, timeout=10)
            self.__register_for_updates()
        except rospy.ROSException:
            rospy.loginfo(
                'The following knowledge base node is currently not present. Connection will be established later: ' + knowledge_base_name)

    def __register_for_updates(self):
        """
        registers at knowledge base for updates of facts, which match the pattern of this instance
        """
        register_for_updates_services = rospy.ServiceProxy(self.__knowledge_base_update_subscriber_service_name,
                                                           UpdateSubscribe)
        response = register_for_updates_services(self.__pattern)
        rospy.Subscriber(response.add_topic_name, Empty, self.__handle_add_update)
        rospy.Subscriber(response.remove_topic_name, FactRemoved, self.__handle_remove_update)
        self.update_state_manually()
        self.__initialized = True
        rospy.logdebug('Connected to knowledge base: '+self.__knowledge_base_name)

    def __handle_add_update(self, fact_added):
        """
        handles message, that a matching fact was added
        :param fact_added: empty message
        """
        self.__contained_facts.append(fact_added.fact)

    def __handle_remove_update(self, fact_removed):
        """
        handles message, that a matching fact was removed
        :param fact_removed: FactRemoved, as defined ROS message
        """
        self.__contained_facts.remove(fact_removed.fact)
        assert self.does_fact_exists() == fact_removed.another_matching_fact_exists

    def update_state_manually(self):
        """
        requests in knowledge base, whether a matching state exists
        :return: whether matching fact exists
        """
        rospy.wait_for_service(self.__all_service_name)
        all_service = rospy.ServiceProxy(self.__all_service_name, All)
        self.__contained_facts = all_service(self.__pattern).facts
        return self.does_fact_exists()

    def __ensure_initialization(self):
        if not self.__initialized:
            rospy.loginfo('Wait for knowledge base service: ' + self.__knowledge_base_update_subscriber_service_name)
            rospy.wait_for_service(self.__knowledge_base_update_subscriber_service_name)
            self.__register_for_updates()


    def does_fact_exists(self):
        """
        :return: current cached value
        """
        self.__ensure_initialization()
        return not(len(self.__contained_facts)==0)

    def get_all_matching_facts(self):
        self.__ensure_initialization()
        return self.__contained_facts
