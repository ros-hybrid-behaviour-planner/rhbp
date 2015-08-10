#!/usr/bin/python
'''
Created on 13.04.2015

@author: stephan
'''
import rospy
from buildingBlocks.managers import Manager

if __name__ == '__main__':
    m = Manager(activationThreshold = 21, prefix = "sim")
    rate = rospy.Rate(1) # 1hz
    while(True):
        m.step()
        rate.sleep()