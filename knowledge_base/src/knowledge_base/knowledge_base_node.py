#! /usr/bin/env python2
"""
Created on 07.12.2016

@author: rieger
"""
import sys

import rospy

from knowledge_base_manager import KnowledgeBase

if __name__ == '__main__':
    rospy.init_node(KnowledgeBase.DEFAULT_NAME, log_level=rospy.DEBUG)
    node_name = None
    for arg in sys.argv:
        if (arg.startswith('__name:=')):
            node_name = arg[len('__name:='):]
    # Design decision to allow using default name from launch files
    if (node_name is None) or (node_name == 'None'):
        node_name = KnowledgeBase.DEFAULT_NAME
    KnowledgeBase(name=node_name)

    rospy.spin()
