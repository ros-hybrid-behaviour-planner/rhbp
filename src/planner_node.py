#! /usr/bin/env python2
'''
Created on 13.04.2015

@author: stephan
'''
import rospy
from behaviour_components.managers import Manager
import ffp

if __name__ == '__main__':
    m = Manager(logLevel = rospy.INFO, activationThreshold = 21, prefix = "sim")
    rate = rospy.Rate(1) # 1Hz
    while(True):
        m.step()
        pddl = m.fetchPDDL()
        try:
            rospy.loginfo("%s",ffp.plan(pddl[0], pddl[1]))
        except Exception as e:
            rospy.logerr("%s", e)
        rate.sleep()
