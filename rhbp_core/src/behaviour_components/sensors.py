"""
Created on 13.04.2015

@author: wypler, hrabia, rieger
"""

import time
from threading import Lock

import rospy
import re
import traceback
from std_msgs.msg import String, Float64
from utils.ros_helpers import get_topic_type
from utils.topic_listener import TopicListener
from utils.misc import FinalInitCaller, LogFileWriter

from rhbp_core.srv import TopicUpdateSubscribe
from .pddl import create_valid_pddl_name

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.conditions.sensors')


class EncodingConstants(object):
    """
    choose here an appropriate encoding style
    """
    HOT_STATE = "hot_state"  # Chose this for nominal, categorical, and discrete values, e.g. booleans
    NONE_STATE = "none"


class RlExtension(object):
    """
    This Extension can be included in the Sensors. It determines how the true values of the sensors should be used the
    RL-algorithm.
    # Encoding types = [ hot_state , none] see EncodingConstants above
    """
    # TODO state_space does not work for negative numbers!
    def __init__(self, encoding="none", state_space=2, include_in_rl=True):
        self.encoding = encoding
        self.state_space = state_space  # State space is only required for EncodingConstants.HOT_STATE
        self.include_in_rl = include_in_rl  # True if it should be used for learning


class Sensor(object):
    """
    This class represents information necessary to make decisions.
    Although it is not abstract it will be barely useful and should be used as base class for actual
    implementations
    """

    _instanceCounter = 0

    def __init__(self, name=None, optional=False, initial_value=None):
        """
        Constructor
        """
        self._name = name if name else "Sensor_{0}".format(Sensor._instanceCounter)
        self._name = create_valid_pddl_name(self._name)
        self._optional = optional
        self._value = initial_value  # this is what it's all about. Of course, the type and how it is acquired will change depending on the specific sensor
        self._value_update_time = rospy.Time.now()
        self._latestValue = initial_value
        self._latest_value_update_time = self._value_update_time
        self._initial_value = initial_value
        self.rl_extension = RlExtension()
        Sensor._instanceCounter += 1

    def sync(self):
        """
        Keep a explicit copy of the current value
        returns the just stored value
        """
        self._value = self._latestValue
        self._value_update_time = self._latest_value_update_time
        return self._value

    def update(self, newValue):
        """
        This method is to refresh the _latestValue.
        :param newValue: the value to update
        :return:
        """
        self._latestValue = newValue
        self._latest_value_update_time = rospy.Time.now()

    @property
    def value(self):
        return self._value

    @property
    def value_update_time(self):
        """
        Time stamp when the current value was fetched
        :return: ROS time stamp
        """
        return self._value_update_time

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

    def unregister(self):
        """
        Overwrite for any explicit cleanup operations
        """
        pass
        
    def __del__(self):
        self.unregister()


class RawTopicSensor(Sensor):
    """
    This sensor just provides access to the raw ROS message content received on a topic
    """

    __metaclass__ = FinalInitCaller

    def __init__(self, name, topic, message_type=None, initial_value=None, create_log=False, print_updates=False):
        """
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

        super(RawTopicSensor, self).__init__(name=name, initial_value=initial_value)

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
                self._logFile = LogFileWriter(path="", filename=self._name, extension=".log")
                self._logFile.write('{0}\n'.format(self._name))
        else:
            rhbplog.logerr("Could not determine message type of: %s. %s", self._topic_name, traceback.format_stack())

    def subscription_callback(self, msg):
        self.update(msg)
        if self.__print_updates:
            rhbplog.logdebug("%s received sensor message: %s of type %s", self._name, self._latestValue,
                             type(self._latestValue))
        if self._iShouldCreateLog:
            self._logFile.append("{0:f}\t{1}\n".format(rospy.get_time(), self._latestValue))

    @property
    def topic_name(self):
        return self._topic_name

    def unregister(self):
        self._sub.unregister()


class TopicSensor(RawTopicSensor):
    """
    ROS topic sensor that subscribes to the given topic and selects defined attributes (parameter message_attr) from the
    message, default is 'data' that is valid for simple ROS messages of type Bool, Float and Int32 ...
    Name Simple is outdated, and has historical reasons, better use the alias TopicSensor
    """

    ATTRIBUTE_SEPARATOR = '.'
    ATTRIBUTE_INDEX_BEGIN = '['
    ATTRIBUTE_INDEX_END = ']'
    ATTRIBUTE_INDEX_PATTERN = '\\' + ATTRIBUTE_INDEX_BEGIN + '.*\\' + ATTRIBUTE_INDEX_END + '$'

    def __init__(self, topic, name=None, message_type=None, initial_value=None, message_attr='data', create_log=False,
                 print_updates=False):
        """
        :param topic: see :class:RawTopicSensor
        :param name: see :class:RawTopicSensor
        :param message_type: see :class:RawTopicSensor
        :param initial_value: see :class:RawTopicSensor + this value is also used in case attributes are inaccessible
        :param message_attr: the message attribute of the msg to use, to access nested attributes, separate attributes
         by TopicSensor.ATTRIBUTE_SEPARATOR; access tuple/list/dict attribute elements with e.g. 'data[0]'
        :param create_log: see :class:RawTopicSensor
        :param print_updates: see :class:RawTopicSensor
        """
        super(TopicSensor, self).__init__(name=name, topic=topic, message_type=message_type,
                                          initial_value=initial_value, create_log=create_log,
                                          print_updates=print_updates)
        self._message_attr = message_attr
        self._message_attr_original = message_attr # store unchanged version for logging
        if TopicSensor.ATTRIBUTE_INDEX_BEGIN in message_attr:
            self._has_index_attr = True
        else:
            self._has_index_attr = False

        if TopicSensor.ATTRIBUTE_SEPARATOR in message_attr:
            self._message_attr = self._message_attr.split(TopicSensor.ATTRIBUTE_SEPARATOR)
            self._is_nested_attr = True
        else:
            self._is_nested_attr = False

    def _get_index_attribute(self, value, attr):
        """
        advanced version of getattr that allows to access interated attributes, like value[index]
        index is recognized with the opening [, if the attr does not contain [ we fallback to normal getattr
        :param value: the object
        :param attr: the attribute with index to access
        :return: the selected attribute
        """
        if TopicSensor.ATTRIBUTE_INDEX_BEGIN in attr:
            index_elem = re.search(TopicSensor.ATTRIBUTE_INDEX_PATTERN, attr).group()
            attr = attr.replace(index_elem, '')
            index = index_elem[1:-1]  # remove [] from result
            if index.isdigit():
                index = int(index)
                return getattr(value, attr)[index]
        else:  # fallback
            return getattr(value, attr)

    def subscription_callback(self, msg):
        try:
            if self._is_nested_attr:
                msg_value = msg
                for nested_attr in self._message_attr:
                    if self._has_index_attr:  # we use this to avoid complex parsing if not necessary
                        msg_value = self._get_index_attribute(msg_value, nested_attr)
                    else:
                        msg_value = getattr(msg_value, nested_attr)
            else:
                if self._has_index_attr:
                    msg_value = self._get_index_attribute(msg, self._message_attr)
                else:
                    msg_value = getattr(msg, self._message_attr)
        except IndexError:
            rhbplog.logwarn("%s: attribute %s index out of bounds, returning initial value!",
                            self.name, self._message_attr_original)
            msg_value = self._initial_value
        except AttributeError:
            rhbplog.logwarn("%s: MSG has no attribute %s, returning initial value!",
                            self.name, self._message_attr_original)
            msg_value = self._initial_value
        super(TopicSensor, self).subscription_callback(msg_value)


"""
Aliases to keep backward compatibility to class old names
"""
PassThroughTopicSensor = RawTopicSensor

SimpleTopicSensor = TopicSensor


class DynamicSensor(Sensor):
    """
    Sensor, which collects values from all topics, matching a pattern.
    If no valid value exists, the default value is returned.
    """

    __metaclass__ = FinalInitCaller

    def __init__(self, pattern, initial_value=None, optional=False,
                 topic_listener_name=TopicListener.DEFAULT_NAME, name=None,
                 expiration_time_values_of_active_topics=-1., expiration_time_values_of_removed_topics=10.0,
                 subscribe_only_first=False, topic_type=None):
        """
        :param pattern: pattern, which will be used for detect relevant topics. The pattern must be of type str.
                        Additionally the pattern must be a valid regular expression,
                        which is full compatible to the python regex.
                        Example: "/myFamousPrefix/[0-9]*"
        :param initial_value: value, which will be used if no topic exists
        :param optional: see optional parameter of constructor from class Sensor
        :param topic_listener_name: name of topic listener
        :param name: see name parameter of constructor from class Sensor
        :param expiration_time_values_of_active_topics: time in seconds,
                                                        after a value of a still existing topic is outdated
        :param expiration_time_values_of_removed_topics: time in seconds, after a value of a removed topic is outdated
        :param subscribe_only_first: whether this sensor only subscribe to the first matching topic (and no other)
        :param topic_type: (Type) Type of subscribed topics. The sensor will only subscribe to a topics, 
                if the type matches. If topic_type is None, no type check is done
        """
        super(DynamicSensor, self).__init__(name=name, optional=optional, initial_value=initial_value)

        self._last_value = None
        self.__time_of_latest_value = None
        self._default_value = initial_value
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
            return True
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
        :return: whether a is smaller than b
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


class AggregationSensor(Sensor):
    """
    sensor class that allows to specify an aggregation function for multiple other sensors
    The function can either be passed as a function reference or implemented by inheriting from the class and
    overwriting self._aggregate()
    """

    def __init__(self, name, sensors, func=None, publish_aggregate=False, optional=False, initial_value=None):
        """
        :param sensors: list of other sensors to aggregate
        :param func: function that will be used to aggregate the sensor values, sensor values will be passed as a list
        :param publish_aggregate: if set to true the aggregated value will be published on a ROS topic, which name can
               be retrieved with topic_name property
        :return should be an aggregated and normalised function value
        """
        super(AggregationSensor, self).__init__(name=name, optional=optional, initial_value=initial_value)
        self._sensors = sensors
        if func is None:
            self._func = self._aggregate
        else:
            self._func = func

        self._topic_name = self._name + "/aggregate"

        if publish_aggregate:
            self.__pub = rospy.Publisher(self._topic_name, Float64, queue_size=10)
        else:
            self.__pub = None

    def _aggregate(self, sensor_values):
        """
        callback to overwrite if used with inheritance
        :param sensor_values: list of sensor values for aggregation
        :return: aggregated float value
        """
        raise NotImplementedError()
        # return the aggregated values

    def sync(self):

        sensor_values = [sensor.sync() for sensor in self._sensors]

        self.update(newValue=self._func(sensor_values))

        res = super(AggregationSensor, self).sync()

        if self.__pub:
            self.__pub.publish(self._value)

        return res

    @property
    def topic_name(self):
        return self._topic_name

    @property
    def sensors(self):
        return self._sensors
