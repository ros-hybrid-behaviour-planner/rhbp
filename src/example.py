#!/usr/bin/python
'''
Created on 13.04.2015

@author: stephan
'''
from __future__ import division # force floating point division when using plain /
import rospy
from std_msgs.msg import Float32
from buildingBlocks.sensors import TopicSensor
from buildingBlocks.managers import Manager

if __name__ == '__main__':
    # create a Manager
    m = Manager(activationThreshold=21)
    # some random helper variables
    batterySensor = TopicSensor("batteryLevelSensor", "/batteryLevel", Float32)
    batterySensor.update(1.0)
    batteryPub = rospy.Publisher('/batteryLevel', Float32, queue_size=1)
    batteryPub.publish(1.0)
    # do the rest
    rate = rospy.Rate(1) # 10hz
    while(True):
        m.step()
        batteryPub.publish(batterySensor.value - 1/100)
        rate.sleep()