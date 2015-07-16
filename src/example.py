#!/usr/bin/python
'''
Created on 13.04.2015

@author: stephan
'''
from __future__ import division # force floating point division when using plain /
import rospy
from std_msgs.msg import Float32, Bool
from buildingBlocks.conditions import Condition
from buildingBlocks.sensors import TopicSensor
from buildingBlocks.activators import BooleanActivator, LinearActivator
from buildingBlocks.goals import Goal
from buildingBlocks.managers import Manager

if __name__ == '__main__':
    # some random helper variables
    homeSelected = False
    # create a Manager
    m = Manager(activationThreshold=21)
    # creating sensors
    batterySensor = TopicSensor("batteryLevelSensor", "/batteryLevel", Float32)
    flyingSensor = TopicSensor("flyingSensor", "/flying", Bool)
    homeSensor = TopicSensor("homeSensor", "/home", Bool)
    objectsFoundSensor = TopicSensor("objectsFoundSensor", "/objectsFound", Float32)
    mapCoverageSensor = TopicSensor("mapCoverageSensor", "/mapCoverage", Float32)
    batterySensor.update(1.0)
    flyingSensor.update(False)
    homeSensor.update(True)
    objectsFoundSensor.update(0.0)
    mapCoverageSensor.update(0.0)
    batteryPub = rospy.Publisher('/batteryLevel', Float32, queue_size=1)
    flyingPub = rospy.Publisher('/flying', Bool, queue_size=1)
    homePub = rospy.Publisher('/home', Bool, queue_size=1)
    mapCoveragePub = rospy.Publisher('/mapCoverage', Float32, queue_size=1)
    objectsFoundPub = rospy.Publisher('/objectsFound', Float32, queue_size=1)
    batteryPub.publish(1.0)
    flyingPub.publish(False)
    homePub.publish(True)
    mapCoveragePub.publish(0.0)
    objectsFoundPub.publish(0.0)
    isNotFlying = Condition(flyingSensor, BooleanActivator(False), name = "isNotFlyingCondition")
    isAtHome = Condition(homeSensor, BooleanActivator(True), name = "isAtHomeCondition")
    mapComplete = Condition(mapCoverageSensor, LinearActivator(0, 1), name = "mapCompleteCondition")
    objectsFound = Condition(objectsFoundSensor, LinearActivator(0, 1), name = "objectsFoundCondition")
    # setting up goals
    returnHomeGoal = m.addGoal(Goal("returnedHome"))
    returnHomeGoal.addCondition(isNotFlying)
    returnHomeGoal.addCondition(isAtHome)
    completeMapGoal = m.addGoal(Goal("completedMap"))
    completeMapGoal.addCondition(mapComplete)
    objectsFoundGoal = m.addGoal(Goal("objectsFound"))
    objectsFoundGoal.addCondition(objectsFound)
    # do the rest
    rate = rospy.Rate(1) # 10hz
    while(True):
        m.step()
        batteryPub.publish(batterySensor.value - 1/100)
        rate.sleep()