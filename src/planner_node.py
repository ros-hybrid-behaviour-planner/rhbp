#!/usr/bin/python2
'''
Created on 13.04.2015

@author: stephan
'''
import rospy
from behaviour_components.managers import Manager

if __name__ == '__main__':
    m = Manager(activationThreshold = 21, prefix = "sim")
    rate = rospy.Rate(1) # 1Hz
    while(True):
        m.step()
        # m.fetchPDDL()
        rate.sleep()
