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


class TopicListener(object):

    SUBSCRIBE_SERVICE_NAME_POSTFIX = '/Subscribe'

    def __init__(self, node_name, include_regex_into_topic_names=True, prefix=None):
        self.__handler = Master(node_name)
        self.__register_lock = Lock()
        self.__update_topics = {}
        self.__include_regex_into_topic_names = include_regex_into_topic_names
        self.__topic_counter = 0
        if (prefix is None):
            self.__prefix = node_name + '/'
        else:
            self.__prefix = prefix + '/'

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
        if (pattern in self.__update_topics):
            topic_names = self.__update_topics[pattern]
            return TopicUpdateSubscribeResponse(topicNameNewTopic=topic_names[0], topicNameTopicRemoved=topic_names[1], existingTopics = self.__find_matching_topcis(regex))

        base_topic_name = TopicListener.generate_topic_name_for_pattern(self.__prefix, pattern,
                                                                        self.__include_regex_into_topic_names,
                                                                        self.__topic_counter)
        self.__topic_counter += 1
        added_topic_name = base_topic_name + '/TopicAdded'
        removed_topic_name = base_topic_name + '/TopicRemoved'
        self.__update_topics[pattern] = (added_topic_name, removed_topic_name)
        return TopicUpdateSubscribeResponse(topicNameNewTopic=added_topic_name,
                                            topicNameTopicRemoved=removed_topic_name, existingTopics = self.__find_matching_topcis(regex))

    def __subscribe_callback_thread_safe(self, request):
        self.__register_lock.acquire()
        try:
            return self.__subscribe_callback(request)
        finally:
            self.__register_lock.release()
