#! /usr/bin/env python2
'''
Created on 13.04.2015

@author: stephan
'''
import rospy
from behaviour_components.managers import Manager

if __name__ == '__main__':
    m = Manager(logLevel = rospy.DEBUG, activationThreshold = 7, prefix = "sim")
    rate = rospy.Rate(1) # 1Hz
    while(True):
        m.step()
        rate.sleep()
