'''
Created on 13.04.2015

@author: wypler, hrabia, rieger
'''

import time
from threading import Lock

import rospy
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.update_handler import KnowledgeBaseFactCache
from rhbp_core.srv import TopicUpdateSubscribe
from std_msgs.msg import String
from utils.ros_helpers import get_topic_type

from .pddl import create_valid_pddl_name
from .topic_listener import TopicListener


class Sensor(object):
    '''
    This class represents information necessary to make decisions.
    Although it is not abstract it will be barely useful and should be used as base class for actual clever implementations that for planner_node subscribe to ROS topics.
    
    '''

    _instanceCounter = 0

    def __init__(self, name=None, optional=False, initial_value=None):
        '''
        Constructor
        '''
        self._name = name if name else "Sensor_{0}".format(Sensor._instanceCounter)
        self._name = create_valid_pddl_name(self._name)
        self._optional = optional
        self._value = initial_value  # this is what it's all about. Of course, the type and how it is acquired will change depending on the specific sensor
        self._latestValue = initial_value

        Sensor._instanceCounter += 1

    def sync(self):
        '''
        Keep a explicit copy of the current value
        returns the just stored value
        '''
        self._value = self._latestValue
        return self._value

    def update(self, newValue):
        """
        This method is to refresh the _latestValue.
        :param newValue: the value to update
        :return:
        """
        self._latestValue = newValue

    @property
    def value(self):
        return self._value

    @property
    def latestValue(self):
        return self._latestValue

    @property
    def optional(self):
        return self._optional

    @optional.setter
    def optional(self, newValue):
        if not isinstance(newValue, bool):
            rospy.logwarn("Passed non-Bool value to 'optional' attribute of sensor %s. Parameter was %s", self._name,
                          newValue)
        else:
            self._optional = newValue

    def __str__(self):
        return self._name

    def __repr__(self):
        return str(self)

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, newName):
        self._name = newName


class PassThroughTopicSensor(Sensor):
    """
    "PassThrough" because the sensor just forwards the received msg
    """

    def __init__(self, name, topic, message_type=None, initial_value=None, create_log=False, print_updates=True):
        """
        "simple" because apparently only primitive message types like Bool and Float have their actual value in a "data" attribute.
        :param topic: topic name to subscribe to
        :param name: name of the sensor, if None a name is generated from the topic
        :param message_type: if not determined an automatic type determination is attempted, requires the topic already be registered on the master
        :param print_updates: Whether a message should be logged (debug), each time, a new value is received
        """

        if name is None:
            if topic is None:
                raise ValueError("Invalid name and topic")
            else:
                name = create_valid_pddl_name(topic)

        super(PassThroughTopicSensor, self).__init__(name=name, initial_value=initial_value)

        self.__print_updates = print_updates
        self._topic_name = topic
        # if the type is not specified, try to detect it automatically
        if message_type is None:
            message_type = get_topic_type(topic)

        if message_type is not None:
            self._sub = rospy.Subscriber(topic, message_type, self.subscription_callback)
            self._iShouldCreateLog = create_log
            if self._iShouldCreateLog:
                self._logFile = open("{0}.log".format(self._name), 'w')
                self._logFile.write('{0}\n'.format(self._name))
        else:
            rospy.logerr("Could not determine message type of: " + topic)

    def subscription_callback(self, msg):
        self.update(msg)
        if (self.__print_updates):
            rospy.logdebug("%s received sensor message: %s of type %s", self._name, self._latestValue,
                           type(self._latestValue))
        if self._iShouldCreateLog:
            self._logFile.write("{0:f}\t{1}\n".format(rospy.get_time(), self._latestValue))
            self._logFile.flush()

    @property
    def topic_name(self):
        return self._topic_name


class SimpleTopicSensor(PassThroughTopicSensor):
    def __init__(self, topic, name=None, message_type=None, initial_value=None, create_log=False, print_updates=True):
        """
        "simple" because apparently only primitive message types like Bool and Float have their actual value in a "data" attribute.
        """
        super(SimpleTopicSensor, self).__init__(name=name, topic=topic, message_type=message_type,
                                                initial_value=initial_value, create_log=create_log,
                                                print_updates=print_updates)

    def subscription_callback(self, msg):
        super(SimpleTopicSensor, self).subscription_callback(msg.data)


class KnowledgeSensor(Sensor):
    """
    Sensor, which provides information about existance of a fact, which matches the given pattern
    """

    def __init__(self, pattern, optional=False, knowledge_base_name=KnowledgeBase.DEFAULT_NAME, sensor_name=None):
        super(KnowledgeSensor, self).__init__(name=sensor_name, optional=optional, initial_value=None)
        self.__value_cache = KnowledgeBaseFactCache(pattern=pattern, knowledge_base_name=knowledge_base_name)

    def sync(self):
        self.update(self.__value_cache.does_fact_exists())
        super(KnowledgeSensor, self).sync()


class DynamicSensor(Sensor):
    """
    Sensor, which collects values from all topics, matching a pattern
    """

    def __init__(self, pattern, default_value=None, optional=False,
                 topic_listener_name=TopicListener.DEFAULT_NAME, sensor_name=None,
                 expiration_time_values_of_active_topics=-1., expiration_time_values_of_removed_topics=10.0,
                 subscribe_only_first=False):
        """
        :param pattern: pattern, which will be used for detect relevant topics.
        :param default_value: value, which will be used if no topic exists
        :param optional: see optional parameter of constructor from class Sensor
        :param topic_listener_name: name of topic listener
        :param sensor_name: see name parameter of constructor from class Sensor
        :param expiration_time_values_of_active_topics: time in secconds,
                                                        after a value of a still existing topic is outdated
        :param expiration_time_values_of_removed_topics: time in secconds, after a value of a removed topic is outdated
        :param subscribe_only_first: whether this sensor only subscribe to the first matching topic (and no other)
        """
        super(DynamicSensor, self).__init__(name=sensor_name, optional=optional, initial_value=default_value)

        self._default_value = default_value
        self.__valid_values = {}
        self.__values_of_removed_topics = {}
        self.__value_lock = Lock()
        self.__expiration_time_values_of_active_topics = expiration_time_values_of_active_topics
        self.__expiration_time_values_of_removed_topics = expiration_time_values_of_removed_topics
        self.__remaining_allowed_topic_subscribing = 1 if subscribe_only_first else -1
        service_name = topic_listener_name + TopicListener.SUBSCRIBE_SERVICE_NAME_POSTFIX
        rospy.wait_for_service(topic_listener_name + TopicListener.SUBSCRIBE_SERVICE_NAME_POSTFIX)
        subscribe_service = rospy.ServiceProxy(service_name,
                                               TopicUpdateSubscribe)
        subscribe_result = subscribe_service(pattern)
        rospy.Subscriber(subscribe_result.topicNameTopicAdded, String, self.__topic_added_callback)
        rospy.Subscriber(subscribe_result.topicNameTopicRemoved, String, self.__topic_removed)
        for topic_name in subscribe_result.existingTopics:
            self.__subscribe_to_topic(topic_name)

    def __subscribe_to_topic(self, topic_name):
        if (self.__remaining_allowed_topic_subscribing >= 0):
            if (self.__remaining_allowed_topic_subscribing == 0):
                rospy.logdebug('Dont subscribe to topics because already subscribed to another: ' + topic_name)
                return
            self.__remaining_allowed_topic_subscribing -= 1
        rospy.logdebug('Subscribed to: ' + topic_name)
        self.__value_lock.acquire()
        try:
            if (topic_name in self.__values_of_removed_topics):
                if (self.__expiration_value < 0):
                    self.__valid_values[topic_name] = self.__values_of_removed_topics[topic_name]
                self.__values_of_removed_topics.pop(topic_name)
        finally:
            self.__value_lock.release()
        topic_type = get_topic_type(topic_name)
        rospy.Subscriber(topic_name, topic_type, lambda value: self.__value_updated(topic_name, value))

    def __value_updated(self, topic_name, value):
        self.__value_lock.acquire()
        try:
            self.__valid_values[topic_name] = (value, time.time())
        finally:
            self.__value_lock.release()

    @staticmethod
    def __filter_values(time_out, values, current_time):
        if (time_out < 0):
            return values
        bound = current_time - time_out
        result = {}
        for topic_name in values.keys():
            value = values[topic_name]
            if (value[1] >= bound):
                result[topic_name] = value
        return result

    def __calculate_valid_values(self):
        """
            :return: all values, which can be used for calculation of singular value
            """
        current_time = time.time()
        self.__value_lock.acquire()
        try:
            self.__valid_values = DynamicSensor.__filter_values(self.__expiration_time_values_of_active_topics,
                                                                self.__valid_values, current_time)
            valied_values = self.__valid_values.values()
            self.__values_of_removed_topics = DynamicSensor.__filter_values(
                self.__expiration_time_values_of_removed_topics, self.__values_of_removed_topics, current_time)
            values_of_removed_topics = self.__values_of_removed_topics.values()
        finally:
            self.__value_lock.release()

        result = []
        result.extend(valied_values)
        result.extend(values_of_removed_topics)
        result = sorted(result, key=lambda t: t[1], reverse=True)
        return map(lambda p: p[0], result)

    def __topic_added_callback(self, name_message):
        self.__subscribe_to_topic(name_message.data)

    def _aggregate_values(self, values):
        """
        :param values: values, as received from the topics (e.g. ROS messages)
        :return: singular value, which will be given to user of this sensor
        """
        if (len(values) == 0):
            return self._default_value

        return values[0]

    def __topic_removed(self, name_message):
        """
        Mark last value of this topic as possibly outdated
        :param name_message: name of removed topic
        """
        topic_name = name_message.data
        self.__value_lock.acquire()
        try:
            if (topic_name in self.__valid_values):
                self.__values_of_removed_topics[topic_name] = self.__valid_values[topic_name]
                self.__valid_values.pop(topic_name)
        finally:
            self.__value_lock.release()

    def sync(self):
        values = self.__calculate_valid_values()
        aggregated_value = self._aggregate_values(values)
        self.update(aggregated_value)
        super(DynamicSensor, self).sync()

class KnowledgeFactSensor(Sensor):
    """
    Sensor, which provides information about a searched fact; returns list of
    all matching facts
    """
    def __init__(self, pattern, optional=False, knowledge_base_name=None,
                 name=None, initial_value=None):
        super(KnowledgeFactSensor, self).__init__(name=name, optional=optional,
                                                  initial_value=initial_value)
        self.__value_cache = KnowledgeBaseFactCache(pattern=pattern,
                                                    knowledge_base_name=
                                                    knowledge_base_name)
    def sync(self):
        self.update(self.__value_cache.get_all_matching_facts())
        super(KnowledgeFactSensor, self).sync()

