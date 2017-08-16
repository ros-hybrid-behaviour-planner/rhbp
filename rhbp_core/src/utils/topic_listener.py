#! /usr/bin/env python2
'''
Created on 01.03.2017

@author: rieger
'''
import re
import sys
from threading import Lock

import rospy
from rosgraph.masterapi import Master
from std_msgs.msg import String

from rhbp_core.srv import TopicUpdateSubscribe, TopicUpdateSubscribeResponse

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager()

class TopicListener(object):
    """
    Service that allows subscribing for updates about added or removed topics
    Provides a method, which checks for new or removed topics and informs subscribers about.
    The topic listener is usually started as own node, see main below. The clients of this service
    register through a service interface and are updated by a topic of new topcis that match their pattern.
    """

    DEFAULT_NAME = 'TopicListenerNode'

    SUBSCRIBE_SERVICE_NAME_POSTFIX = '/Subscribe'

    def __init__(self, include_regex_into_topic_names=True, prefix=DEFAULT_NAME):
        """
        :param include_regex_into_topic_names: Whether names of update topic should contain the pattern. Just for better debugging. Has no influence on functionality
        :param prefix: address of the topic listener. The address is used for the subscribe service and all update topics.
        """
        self.__handler = Master(rospy.get_name())  # get access to the ROS master to use the topic directoy
        self.__lock = Lock()
        self.__update_topics = {}
        self.__include_regex_into_topic_names = include_regex_into_topic_names
        self.__topic_counter = 0
        self.__subscribed_regular_expressions = []
        self.__prefix = prefix
        self.__subscribe_service = rospy.Service(self.__prefix + TopicListener.SUBSCRIBE_SERVICE_NAME_POSTFIX,
                                                 TopicUpdateSubscribe,
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
        :param include_pattern: whether the regex should occur in the topic name
        :param counter: unique number for this topic name
        :return: topic name
        """
        topic_name = pattern + 'Topic' + str(counter)
        if include_pattern:
            topic_name += '_' + re.sub(r'[^a-zA-Z0-9]', '', str(regex))
        return topic_name

    def __find_matching_topcis(self, regex):
        """
        :param regex: (regular expression) regex for matching the topics
        :return: names of all existing topics, which match the regular expression
        """
        matching_topics = []
        for topic_name in self.__existing_topics:
            if (regex.match(topic_name)):
                matching_topics.append(topic_name)
        return tuple(matching_topics)

    def __subscribe_callback_thread_safe(self, request):
        """
        Creates topics for the requested pattern and returns all already existing topic names, matching the pattern
        :param request: (TopicUpdateSubscribe) subscribe request.
                        The pattern must be a regular expression, compatible to pythons regular expressions
        :return: service response
        """
        with self.__lock:
            pattern = request.regex
            regex = re.compile(pattern)
            if (regex in self.__update_topics):
                topic_names = self.__update_topics[regex]
                return TopicUpdateSubscribeResponse(topicNameTopicAdded=topic_names[0].name,
                                                    topicNameTopicRemoved=topic_names[1].name,
                                                    existingTopics=self.__find_matching_topcis(regex))

            self.__subscribed_regular_expressions.append(regex)
            base_topic_name = TopicListener.generate_topic_name_for_pattern(self.__prefix + '/Topics/', pattern,
                                                                            self.__include_regex_into_topic_names,
                                                                            self.__topic_counter)
            self.__topic_counter += 1
            added_topic_name = base_topic_name + '/TopicAdded'
            added_topic = rospy.Publisher(added_topic_name, String, queue_size=10)
            removed_topic_name = base_topic_name + '/TopicRemoved'
            removed_topic = rospy.Publisher(removed_topic_name, String, queue_size=10)
            rospy.sleep(0.1)
            self.__update_topics[regex] = (added_topic, removed_topic)

            rhbplog.logdebug('subscribed for pattern: ' + pattern)
            return TopicUpdateSubscribeResponse(topicNameTopicAdded=added_topic_name,
                                                topicNameTopicRemoved=removed_topic_name,
                                                existingTopics=self.__find_matching_topcis(regex))

    def __inform_about_topic_change(self, changed_topics, inform_about_added_topics):
        """
        Informs all subscribed topics about the topic changes.
        Each client is only informed about changes of matching topics
        :param changed_topics: names of all changed topics
        :param inform_about_added_topics: whether the changed topics are added (or removed).
                                          Is used for decision about used topic
        """
        if (inform_about_added_topics):
            index_of_topic_in_pair = 0
        else:
            index_of_topic_in_pair = 1
        for regex in self.__subscribed_regular_expressions:
            for topic in changed_topics:
                if (regex.match(topic)):
                    self.__update_topics[regex][index_of_topic_in_pair].publish(topic)

    def __inform_about_added_topics(self, added_topics):
        self.__inform_about_topic_change(added_topics, True)

    def __inform_about_removed_topics(self, removed_topics):
        self.__inform_about_topic_change(removed_topics, False)

    def check(self):
        """
        Update method that tracks known and new topics
        It triggers the pattern comparision in case of new
        available topics
        """
        with self.__lock:
            expected_topics = list(self.__existing_topics)
            self.__existing_topics = []
            added_topics = []
            topics = self.__handler.getPublishedTopics('')
            for topic in topics:
                topic_name = topic[0]
                # cache and update existing and new topics
                self.__existing_topics.append(topic_name)
                if (topic_name in expected_topics):
                    expected_topics.remove(topic_name)
                else:
                    added_topics.append(topic_name)
            self.__inform_about_added_topics(added_topics)
            if (added_topics):
                rhbplog.logdebug('New Topics: ' + str(added_topics))

            if (expected_topics):
                rhbplog.logdebug('Removed Topics: ' + str(expected_topics))
            self.__inform_about_removed_topics(expected_topics)


if __name__ == '__main__':

    node_name = None
    for arg in sys.argv:
        if (arg.startswith('__name:=')):
            node_name = arg[len('__name:='):]
    # Design decision to allow using default name from launch files
    if (node_name is None) or (node_name == 'None'):
        node_name = TopicListener.DEFAULT_NODE_NAME
    rospy.init_node(node_name, log_level=rospy.DEBUG)

    rate = rospy.Rate(rospy.get_param("~checkFrequency", 1))
    listener = TopicListener()
    while (not rospy.is_shutdown()):
        listener.check()
        rate.sleep()
