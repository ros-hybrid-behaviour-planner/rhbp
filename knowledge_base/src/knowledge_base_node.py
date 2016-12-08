#! /usr/bin/env python2
'''
Created on 07.12.2016

@author: phillip
'''
import sys
import rospy
from knowledge_base_manager import KnowledgeBase
from knowledge_base.srv import Exists, ExistsResponse
from knowledge_base.msg import Push

if __name__ == '__main__':
    if (len(sys.argv)<2):
        nodeName='knowledgeBaseNode'
    else:
        nodeName='knowledgeBase/' + sys.argv[1]
    rospy.init_node(nodeName, log_level = rospy.WARN)
    KnowledgeBase(nodeName)

    rospy.spin()

