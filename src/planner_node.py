#! /usr/bin/env python2
'''
Created on 13.04.2015

@author: stephan
'''
import rospy
from behaviour_components.managers import Manager

if __name__ == '__main__':
    m = Manager(logLevel = rospy.INFO, activationThreshold = 21, prefix = "sim")
    rate = rospy.Rate(.5) # .5Hz
    while(True):
        m.step()
        rate.sleep()
