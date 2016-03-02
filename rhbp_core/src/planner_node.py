#! /usr/bin/env python2
'''
Created on 13.04.2015

@author: stephan
'''
import rospy
from behaviour_components.managers import Manager

if __name__ == '__main__':
    rospy.init_node('behaviourPlannerManager', log_level = rospy.DEBUG)
    m = Manager(activationThreshold = rospy.get_param("~activationThreshold", 7.0), prefix = rospy.get_param("~prefix", "sim")) 
    rate = rospy.Rate(rospy.get_param("~frequency", 1))
    while(True):
        m.step()
        rate.sleep()
