'''
Created on 13.04.2015

@author: wypler, hrabia, rieger
'''

import time
from threading import Lock

import rospy
from std_msgs.msg import String
from utils.ros_helpers import get_topic_type
from utils.topic_listener import TopicListener
from utils.misc import FinalInitCaller, LogFileWriter

from rhbp_core.srv import TopicUpdateSubscribe
from .pddl import create_valid_pddl_name

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.conditions.sensors')

class Sensor(object):
    '''
    This class represents information necessary to make decisions.
    Although it is not abstract it will be barely useful and should be used as base class for actual
    implementations
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
            rhbplog.logwarn("Passed non-Bool value to 'optional' attribute of sensor %s. Parameter was %s", self._name,
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

    __metaclass__ = FinalInitCaller

    def __init__(self, name, topic, message_type=None, initial_value=None, create_log=False, print_updates=False):
        """
        "simple" because apparently only primitive message types like Bool and Float have their actual value in a "data" attribute.
        :param topic: topic name to subscribe to
        :param name: name of the sensor, if None a name is generated from the topic
        :param message_type: if not determined an automatic type determination is attempted, requires the topic already be registered on the ROS master
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
        self._iShouldCreateLog = create_log
        self._message_type = message_type

    def final_init(self):
        """
        Ensure registration after the entire initialisation (including sub classes) is done
        """
        # if the type is not specified, try to detect it automatically
        if self._message_type is None:
            self._message_type = get_topic_type(self._topic_name)

        if self._message_type is not None:
            self._sub = rospy.Subscriber(self._topic_name, self._message_type, self.subscription_callback)

            if self._iShouldCreateLog:
                self._logFile = LogFileWriter(path="",filename=self._name,extension=".log")
                self._logFile.write('{0}\n'.format(self._name))
        else:
            rhbplog.logerr("Could not determine message type of: " + self._topic_name)

    def subscription_callback(self, msg):
        self.update(msg)
        if (self.__print_updates):
            rhbplog.logdebug("%s received sensor message: %s of type %s", self._name, self._latestValue,
                           type(self._latestValue))
        if self._iShouldCreateLog:
            self._logFile.append("{0:f}\t{1}\n".format(rospy.get_time(), self._latestValue))

    @property
    def topic_name(self):
        return self._topic_name

    def __del__(self):
        self._sub.unregister()


class SimpleTopicSensor(PassThroughTopicSensor):
    """
    ROS topic sensor that subscribes to the given topic and provides defined primitive (for this reason simple)
    attributes (parameter message_attr) from the message,
    default is 'data' that is valid for simple ROS messages of type Bool, Float and Int32 ...
    """

    def __init__(self, topic, name=None, message_type=None, initial_value=None, message_attr='data', create_log=False, print_updates=False):
        """
        :param topic: see :class:PassThroughTopicSensor
        :param name: see :class:PassThroughTopicSensor
        :param message_type: see :class:PassThroughTopicSensor
        :param initial_value: see :class:PassThroughTopicSensor
        :param message_attr: the message attribute of the msg to use
        :param create_log: see :class:PassThroughTopicSensor
        :param print_updates: see :class:PassThroughTopicSensor
        """
        super(SimpleTopicSensor, self).__init__(name=name, topic=topic, message_type=message_type,
                                                initial_value=initial_value, create_log=create_log,
                                                print_updates=print_updates)
        self._message_field = message_attr

    def subscription_callback(self, msg):
        msg_value = getattr(msg, self._message_field)
        super(SimpleTopicSensor, self).subscription_callback(msg_value)


class DynamicSensor(Sensor):
    """
    Sensor, which collects values from all topics, matching a pattern.
    If no valid value exists, the default value is returned.
    """

    __metaclass__ = FinalInitCaller

    def __init__(self, pattern, default_value=None, optional=False,
                 topic_listener_name=TopicListener.DEFAULT_NAME, name=None,
                 expiration_time_values_of_active_topics=-1., expiration_time_values_of_removed_topics=10.0,
                 subscribe_only_first=False, topic_type=None):
        """
        :param pattern: pattern, which will be used for detect relevant topics. The pattern must be of type str.
                        Additionally the pattern must be a valid regular expression,
                        which is full compatible to the python regex.
                        Example: "/myFamousPrefix/[0-9]*"
        :param default_value: value, which will be used if no topic exists
        :param optional: see optional parameter of constructor from class Sensor
        :param topic_listener_name: name of topic listener
        :param sensor_name: see name parameter of constructor from class Sensor
        :param expiration_time_values_of_active_topics: time in secconds,
                                                        after a value of a still existing topic is outdated
        :param expiration_time_values_of_removed_topics: time in secconds, after a value of a removed topic is outdated
        :param subscribe_only_first: whether this sensor only subscribe to the first matching topic (and no other)
        :param topic_type: (Type) Type of subscribed topics. The sensor will only subscribe to a topics, 
                if the type matches. If topic_type is None, no type check is done
        """
        super(DynamicSensor, self).__init__(name=name, optional=optional, initial_value=default_value)

        self._last_value = None
        self.__time_of_latest_value = None
        self._default_value = default_value
        self._min_value = None
        self._max_value = None
        self.__valid_values = {}
        self.__values_of_removed_topics = {}
        self.__value_lock = Lock()
        self.__expiration_time_values_of_active_topics = expiration_time_values_of_active_topics
        self.__expiration_time_values_of_removed_topics = expiration_time_values_of_removed_topics
        self.__remaining_allowed_topic_subscribing = 1 if subscribe_only_first else -1
        self.__topic_type = topic_type
        self._topic_listener_name = topic_listener_name
        self._pattern = pattern

    def final_init(self):
        service_name = self._topic_listener_name + TopicListener.SUBSCRIBE_SERVICE_NAME_POSTFIX
        rospy.wait_for_service(self._topic_listener_name + TopicListener.SUBSCRIBE_SERVICE_NAME_POSTFIX)
        subscribe_service = rospy.ServiceProxy(service_name,
                                               TopicUpdateSubscribe)
        subscribe_result = subscribe_service(self._pattern)
        rospy.Subscriber(subscribe_result.topicNameTopicAdded, String, self.__topic_added_callback)
        rospy.Subscriber(subscribe_result.topicNameTopicRemoved, String, self.__topic_removed)
        for topic_name in subscribe_result.existingTopics:
            self.__subscribe_to_topic(topic_name)

    def _matches_topic_type(self, topic_name, topic_type):
        """
        Checks, whether the type of the topic has the correct type
        :param topic_type: (Type) type of topic, which is checked
        :param topic_name: name of topic
        :return: whether the topic type is valid
        """
        if (self.__topic_type is None):
            return True;
        return self.__topic_type == topic_type

    def __subscribe_to_topic(self, topic_name):
        topic_type = get_topic_type(topic_name)
        if (not self._matches_topic_type(topic_name, topic_type)):
            return
        if (self.__remaining_allowed_topic_subscribing >= 0):
            if (self.__remaining_allowed_topic_subscribing == 0):
                rhbplog.logdebug('Dont subscribe to topics because already subscribed to another: ' + topic_name)
                return
            self.__remaining_allowed_topic_subscribing -= 1
        rhbplog.logdebug('Subscribed to: ' + topic_name)
        self.__value_lock.acquire()
        try:
            if (topic_name in self.__values_of_removed_topics):
                if (self.__expiration_value < 0):
                    self.__valid_values[topic_name] = self.__values_of_removed_topics[topic_name]
                self.__values_of_removed_topics.pop(topic_name)
        finally:
            self.__value_lock.release()
        rospy.Subscriber(topic_name, topic_type, lambda value: self.__value_updated(topic_name, value))

    def _value_updated(self, topic, value, time_stamp):
        """
        Stores the latest value. Can also be used as hook method for scenario dependent calculations (e.g. min, max)
        :param topic: name of the topic, where the value was received
        :param value: value as received
        :param time_stamp: time of receiving. see time.time()
        """
        if (self._last_value is None):
            self._last_value = value
            self.__time_of_latest_value = time_stamp
        elif (time_stamp > self.__time_of_latest_value):
            self._last_value = value
            self.__time_of_latest_value = time_stamp
        if (self._min_value is None or self._smaller_than(value, self._min_value)):
            self._min_value = value
        if (self._max_value is None or self._smaller_than(self._max_value, value)):
            self._max_value = value

    def __value_updated(self, topic_name, value):
        with self.__value_lock:
            time_stamp = time.time()
            self.__valid_values[topic_name] = (value, time_stamp)
            self._value_updated(topic_name, value, time_stamp)

    @staticmethod
    def __filter_values(time_out, values, current_time):
        """
        :param time_out: time out in seconds
        :param values: (Dictonary) topic names (key), associated with their latest values. The values are a tuple and have the form of (value, receive time)
        :param current_time: time stamp, which is used for comparison
        :return: all values, which was received after current_time-timeout
        """
        if (time_out < 0):
            return values
        bound = current_time - time_out
        result = {}
        for topic_name in values.keys():
            value = values[topic_name]
            if (value[1] >= bound):
                result[topic_name] = value
        return result

    def _smaller_than(self, a, b):
        """
        :param a: a received message
        :param b: another received message
        :return: wether a is smaller than b
        """
        if (hasattr(a, 'data') and hasattr(b, 'data')):
            return a.data < b.data
        # In this case, the result of comparison is probably wrong
        rhbplog.logwarn(str(type(a)) + ' and ' + str(
            type(b)) + ' have not the attribute data. Unsafety default implementation is used')
        return a < b

    def __calculate_valid_values(self):
        """
            :return: all values, which can be used for calculation of singular value in the later aggregation step
            """
        current_time = time.time()
        self.__value_lock.acquire()
        try:
            self.__valid_values = DynamicSensor.__filter_values(self.__expiration_time_values_of_active_topics,
                                                                self.__valid_values, current_time)
            valid_values = self.__valid_values.values()
            self.__values_of_removed_topics = DynamicSensor.__filter_values(
                self.__expiration_time_values_of_removed_topics, self.__values_of_removed_topics, current_time)
            values_of_removed_topics = self.__values_of_removed_topics.values()
        finally:
            self.__value_lock.release()

        result = []
        result.extend(valid_values)
        result.extend(values_of_removed_topics)
        result = sorted(result, key=lambda t: t[1], reverse=True)
        return map(lambda p: p[0], result)

    def __topic_added_callback(self, name_message):
        self.__subscribe_to_topic(name_message.data)

    def _convert_ros_message_to_output_format(self, to_convert):
        """
        Since the result of aggregation may has a different type than the received message,
        this method converts a ros message to output format.
        Is used, e.g. to convert the last received message to the target tye.
        :param to_convert:
        :return: a value, which is an instance of the output type
        """
        return to_convert

    def _aggregate_values(self, values):
        """
        This default implementation either returns the default value if no values are
        available or it returns the latest value
        :param values: values, as received from the topics (e.g. ROS messages)
        :return: aggregated singular value, which will be given to user of this sensor
        """
        if (len(values) == 0):
            if (not self._last_value is None):
                return self._convert_ros_message_to_output_format(self._last_value)
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
