#!/usr/bin/python
'''
Created on 13.04.2015

@author: stephan
'''
from __future__ import division # force floating point division when using plain /
import rospy
import random # because why not?
from buildingBlocks.conditions import Condition, Disjunction, Conjunction
from buildingBlocks.sensors import Sensor
from buildingBlocks.behaviour import Behaviour
from buildingBlocks.activators import BooleanActivator, LinearActivator
from buildingBlocks.goals import Goal
from buildingBlocks.managers import Manager


class MoveBehaviour(Behaviour):
    def __init__(self, *args, **kwargs):
        super(MoveBehaviour, self).__init__(*args, **kwargs)
        
    def action(self):
        global homeSensor
        global homeSelected
        global mapCoverageSensor
        global targetSelectedSensor
        global objectsFoundSensor
        homeSensor.update(homeSelected)
        if not homeSelected:
            mapImprovement = random.random() * .1
            for i in range(10):
                if mapImprovement + mapCoverageSensor.value <= 1:
                    mapCoverageSensor.update(mapImprovement + mapCoverageSensor.value)
                    break
                else:
                    mapImprovement = random.random() * .1
            objectImprovement = random.random() * .1
            for i in range(10):
                if objectImprovement + objectsFoundSensor.value <= 1:
                    objectsFoundSensor.update(objectImprovement + objectsFoundSensor.value)
                    break
                else:
                    objectImprovement = random.random() * .1
        targetSelectedSensor.update(False)
        return False


if __name__ == '__main__':
    # some random helper variables
    homeSelected = False
    # create a Manager
    m = Manager(activationThreshold=21)
    """
    # creating sensors
    batterySensor = m.addSensor(Sensor("batteryLevelSensor"))
    flyingSensor = m.addSensor(Sensor("flyingSensor"))
    homeSensor = m.addSensor(Sensor("homeSensor"))
    targetSelectedSensor = m.addSensor(Sensor("targetSelectedSensor"))
    objectsFoundSensor = m.addSensor(Sensor("objectsFoundSensor"))
    mapCoverageSensor = m.addSensor(Sensor("mapCoverageSensor"))
    # initial conditions (this is normally done in the simulation loop)
    batterySensor.update(1.0)
    flyingSensor.update(False)
    homeSensor.update(True)
    targetSelectedSensor.update(False)
    objectsFoundSensor.update(0.0)
    mapCoverageSensor.update(0.0)
    # setting up (pre-)conditions
    fullBattery = Condition(batterySensor, LinearActivator(.05, .21), name = "fullBatteryCondition")
    emptyBattery = Condition(batterySensor, LinearActivator(.21, .05), name = "emptyBatteryCondition")
    isFlying = Condition(flyingSensor, BooleanActivator(True), name = "isFlyingCondition")
    isNotFlying = Condition(flyingSensor, BooleanActivator(False), name = "isNotFlyingCondition")
    isAtHome = Condition(homeSensor, BooleanActivator(True), name = "isAtHomeCondition")
    isNotAtHome = Condition(homeSensor, BooleanActivator(False), name = "isNotAtHomeCondition")
    targetSelected = Condition(targetSelectedSensor, BooleanActivator(True), name = "targetSelectedCondition")
    targetNotSelected = Condition(targetSelectedSensor, BooleanActivator(False), name = "targetNotSelectedCondition")
    objectsFound = Condition(objectsFoundSensor, LinearActivator(0, 1), name = "objectsFoundCondition")
    objectsNotFound = Condition(objectsFoundSensor, LinearActivator(1, .7), name = "objectsNotFoundCondition")
    mapComplete = Condition(mapCoverageSensor, LinearActivator(0, 1), name = "mapCompleteCondition")
    mapIncomplete = Condition(mapCoverageSensor, LinearActivator(1, .7), name = "mapIncompleteCondition")
    # setting up behaviours
    startBehaviour = m.addBehaviour(Behaviour("startBehaviour", correlations = {flyingSensor: 1.0}))
    def startAction():
        flyingSensor.update(True)
        return False
    startBehaviour.action = startAction
    startBehaviour.addPrecondition(isNotFlying)
    startBehaviour.addPrecondition(fullBattery)
    startBehaviour.addPrecondition(mapIncomplete)
    startBehaviour.addPrecondition(objectsNotFound)
    landBehaviour = m.addBehaviour(Behaviour("landBehaviour", correlations = {flyingSensor: -1.0}))
    def landAction():
        flyingSensor.update(False)
        return False
    landBehaviour.action = landAction
    landBehaviour.addPrecondition(isFlying)
    landBehaviour.addPrecondition(isAtHome)
    jobDone = Conjunction(objectsFound, mapComplete, name="jobDone")
    landBehaviour.addPrecondition(Disjunction(emptyBattery, jobDone, name = "landingCriteria"))
    goHomeBehaviour = m.addBehaviour(Behaviour("goHomeBehaviour", correlations = {targetSelectedSensor: 1.0}))
    def goHomeAction():
        targetSelectedSensor.update(True)
        global homeSelected
        homeSelected = True
        return False
    goHomeBehaviour.action = goHomeAction
    component1 = Conjunction(isNotAtHome, emptyBattery, name="awayAndBatteryEmpty")
    goHomeBehaviour.addPrecondition(Disjunction(component1, jobDone, name="goHomeMix"))
    selectTargetBehaviour = m.addBehaviour(Behaviour("selectTargetBehaviour", correlations = {targetSelectedSensor: 1.0}))
    def selectTargetActionAction():
        targetSelectedSensor.update(True)
        return False
    selectTargetBehaviour.action = selectTargetActionAction
    selectTargetBehaviour.readyThreshold = 0.42
    selectTargetBehaviour.addPrecondition(fullBattery)
    selectTargetBehaviour.addPrecondition(isFlying)
    selectTargetBehaviour.addPrecondition(targetNotSelected)
    selectTargetBehaviour.addPrecondition(Disjunction(objectsNotFound, mapIncomplete, name = "noMapNorObjectsDisjunction"))
    moveBehaviour = m.addBehaviour(MoveBehaviour("moveBehaviour", correlations = {homeSensor: -.1, mapCoverageSensor: 0.5, objectsFoundSensor: 0.5, targetSelectedSensor: -1.0}))      
    moveBehaviour.addPrecondition(isFlying)
    moveBehaviour.addPrecondition(targetSelected)
    # setting up goals
    returnHomeGoal = m.addGoal(Goal("returnedHome"))
    returnHomeGoal.addCondition(isNotFlying)
    returnHomeGoal.addCondition(isAtHome)
    completeMapGoal = m.addGoal(Goal("completedMap"))
    completeMapGoal.addCondition(mapComplete)
    objectsFoundGoal = m.addGoal(Goal("objectsFound"))
    objectsFoundGoal.addCondition(objectsFound)
    """
    rate = rospy.Rate(1) # 10hz
    while(True):
        m.step()
#        batterySensor.update(batterySensor.value - 1/100)
        rate.sleep()