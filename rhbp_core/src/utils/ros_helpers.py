"""
This module contains several ROS related helper functions
The functions need an already initialized ros node if nothing else is mentioned

moduleauthor:: hrabia
"""

import rosgraph
import roslib.names
import rospy
from rospy.exceptions import ROSException
from genpy.message import get_message_class


def get_topic_type_info(topic_name):
    """
    subroutine for getting topic type information
    (nearly identical to rostopic._get_topic_type, except it returns rest of name instead of fn)
    :param topic_name: topic name of interest
    :type topic_name: str
    :returns: topic type, real topic name, and rest of name referenced
      if the topic points to a field within a topic, e.g. /rosout/msg, ``str, str, str``
    """
    try:
        node_name = rospy.get_name()
        master = rosgraph.Master(node_name) 
        val = master.getTopicTypes()
    except:
        raise ROSException("unable to get list of topics from master")
    matches = [(t, t_type) for t, t_type in val if t == topic_name or topic_name.startswith(t + '/')]
    if matches:
        t, t_type = matches[0]
        if t_type == roslib.names.ANYTYPE:
            return None, None, None
        if t_type == topic_name:
            return t_type, None
        return t_type, t, topic_name[len(t):]
    else:
        return None, None, None

def get_topic_type(topic_name):
    """
    Get the type/class of a topic
    :param topic_name: topic name of interest
    :type topic_name: str
    :returns topic type
    """
    topic_info = get_topic_type_info(topic_name)
    if topic_info[0] is None:
        return None
    else:
        return get_message_class(topic_info[0], True)
