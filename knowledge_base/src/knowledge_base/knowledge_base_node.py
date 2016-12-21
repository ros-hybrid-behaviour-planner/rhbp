#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: phillip
'''
import rospy

from knowledge_base_manager import KnowledgeBase

if __name__ == '__main__':
    nodeName = 'knowledgeBaseNode'
    rospy.init_node(nodeName, log_level=rospy.DEBUG)
    KnowledgeBase(nodeName)

    rospy.spin()
