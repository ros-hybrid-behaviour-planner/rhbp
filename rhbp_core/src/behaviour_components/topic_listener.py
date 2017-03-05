#! /usr/bin/env python2
'''
Created on 01.03.2017

@author: rieger
'''
import re
from threading import Lock

import rospy
from rhbp_core.srv import TopicUpdateSubscribe,TopicUpdateSubscribeResponse
from rosgraph.masterapi import Master
from std_msgs.msg import String
import sys

class TopicListener(object):

    DEFAULT_NODE_NAME= 'TopicListenerNode'

    SUBSCRIBE_SERVICE_NAME_POSTFIX = '/Subscribe'

    def __init__(self, node_name, include_regex_into_topic_names=True, prefix=None):
        self.__handler = Master(node_name)
        self.__lock = Lock()
        self.__update_topics = {}
        self.__include_regex_into_topic_names = include_regex_into_topic_names
        self.__topic_counter = 0
        self.__subscribed_regular_expressions = []
        if (prefix is None):
            self.__prefix = node_name
        else:
            self.__prefix = prefix
        self.__subscribe_service = rospy.Service(self.__prefix + TopicListener.SUBSCRIBE_SERVICE_NAME_POSTFIX, TopicUpdateSubscribe,
                                                  self.__subscribe_callback_thread_safe)
        self.__existing_topics = []

    def __del__(self):
        """
            closes all services
        """
        self.__subscribe_service.shutdown()

    @staticmethod
    def generate_topic_name_for_pattern(pattern, regex, include_pattern, counter):
        """
        generates topic name for given regex
        :param prefix: prefix for topic names
        :param pattern: regex (type string)
        :return: topic name
        """
        topic_name = pattern + 'Topic' + str(counter)
        if include_pattern:
            topic_name += '_' + re.sub(r'[^a-zA-Z0-9]', '', str(regex))
        return topic_name

    def __find_matching_topcis(self, regex):
        matching_topics = []
        for topic_name in self.__existing_topics:
            if (regex.match(topic_name)):
                matching_topics.append(topic_name)
        return tuple(matching_topics)


    def __subscribe_callback(self, request):
        pattern = request.regex
        regex = re.compile(pattern)
        if (regex in self.__update_topics):
            topic_names = self.__update_topics[regex]
            return TopicUpdateSubscribeResponse(topicNameNewTopic=topic_names[0], topicNameTopicRemoved=topic_names[1], existingTopics = self.__find_matching_topcis(regex))

        self.__subscribed_regular_expressions.append(regex)
        base_topic_name = TopicListener.generate_topic_name_for_pattern(self.__prefix + '/', pattern,
                                                                        self.__include_regex_into_topic_names,
                                                                        self.__topic_counter)
        self.__topic_counter += 1
        added_topic_name = base_topic_name + '/TopicAdded'
        added_topic =rospy.Publisher(added_topic_name,String, queue_size=10)
        removed_topic_name = base_topic_name + '/TopicRemoved'
        removed_topic =rospy.Publisher(removed_topic_name,String, queue_size=10)
        rospy.sleep(1)
        self.__update_topics[regex] = (added_topic, removed_topic)
        return TopicUpdateSubscribeResponse(topicNameNewTopic=added_topic_name,
                                            topicNameTopicRemoved=removed_topic_name, existingTopics = self.__find_matching_topcis(regex))

    def __subscribe_callback_thread_safe(self, request):
        self.__lock.acquire()
        try:
            return self.__subscribe_callback(request)
        finally:
            self.__lock.release()

    def __inform_about_topic_change(self, changed_topics, index_of_topic_in_pair):
        for regex in self.__subscribed_regular_expressions:
            for topic in changed_topics:
                if (regex.match(topic)):
                    self.__update_topics[index_of_topic_in_pair].publish(topic)

    def __inform_about_added_topics(self, added_topics):
        self.__inform_about_topic_change(added_topics,0)

    def __inform_about_removed_topics(self, removed_topics):
        self.__inform_about_topic_change(removed_topics,1)

    def check(self):
        self.__lock.acquire()
        try:
            expected_topics = list(self.__existing_topics)
            self.__existing_topics= []
            added_topics = []
            topics = self.__handler.getPublishedTopics('')
            for topic in topics:
                self.__existing_topics.append(topic)
                if (topic in expected_topics):
                    expected_topics.remove(topic)
                else:
                    added_topics.append(topic)
            self.__inform_about_added_topics(added_topics)
            self.__inform_about_removed_topics(expected_topics)

        finally:
            self.__lock.release()

if __name__ == '__main__':

    node_name = None
    for arg in sys.argv:
        if (arg.startswith('__name:=')):
            node_name = arg[len('__name:='):]
    # Design decision for to allow using default name from launch files
    if (node_name is None) or (node_name == 'None'):
        node_name = TopicListener.DEFAULT_NODE_NAME
    rospy.init_node(node_name, log_level=rospy.DEBUG)

    rate = rospy.Rate(rospy.get_param("~checkFrequency", 1))
    listener = TopicListener(node_name=node_name)
    while (not rospy.is_shutdown()):
        listener.check()
        rate.sleep()