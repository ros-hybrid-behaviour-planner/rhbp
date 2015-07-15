#!/usr/bin/python
'''
Created on 13.04.2015

@author: stephan
'''
from __future__ import division # force floating point division when using plain /
import rospy
from buildingBlocks.conditions import Condition
from buildingBlocks.sensors import Sensor
from buildingBlocks.activators import BooleanActivator, LinearActivator
from buildingBlocks.goals import Goal
from buildingBlocks.managers import Manager

if __name__ == '__main__':
    # some random helper variables
    homeSelected = False
    # create a Manager
    m = Manager(activationThreshold=21)
    # setting up goals
    flyingSensor = Sensor("flyingSensor")
    homeSensor = Sensor("homeSensor")
    mapCoverageSensor = Sensor("mapCoverageSensor")
    objectsFoundSensor = Sensor("objectsFoundSensor")
    flyingSensor.update(False)
    homeSensor.update(True)
    objectsFoundSensor.update(0.0)
    mapCoverageSensor.update(0.0)
    isNotFlying = Condition(flyingSensor, BooleanActivator(False), name = "isNotFlyingCondition")
    isAtHome = Condition(homeSensor, BooleanActivator(True), name = "isAtHomeCondition")
    mapComplete = Condition(mapCoverageSensor, LinearActivator(0, 1), name = "mapCompleteCondition")
    objectsFound = Condition(objectsFoundSensor, LinearActivator(0, 1), name = "objectsFoundCondition")
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
        rate.sleep()