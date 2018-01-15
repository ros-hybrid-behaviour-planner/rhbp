#! /usr/bin/env python2
"""
Created on 07.12.2016

@author: rieger, hrabia
"""
import sys

import rospy

from knowledge_base_manager import KnowledgeBase
from knowledge_base.msg import DiscoverInfo

if __name__ == '__main__':
    rospy.init_node(KnowledgeBase.DEFAULT_NAME, log_level=rospy.DEBUG)
    node_name = None
    for arg in sys.argv:
        if arg.startswith('__name:='):
            node_name = arg[len('__name:='):]
    # Design decision to allow using default name from launch files
    if (node_name is None) or (node_name == 'None'):
        node_name = KnowledgeBase.DEFAULT_NAME
    KnowledgeBase(name=node_name)

    rate = rospy.Rate(rospy.get_param("~discover_frequency", 1))

    discover_pub = rospy.Publisher(KnowledgeBase.DISCOVERY_TOPIC_NAME, DiscoverInfo, queue_size=1)

    while not rospy.is_shutdown():
        if discover_pub.get_num_connections() > 0:
            discover_pub.publish(DiscoverInfo(kb_name=node_name, stamp=rospy.Time.now()))
        rate.sleep()
