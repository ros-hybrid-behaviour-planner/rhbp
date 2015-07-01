#!/usr/bin/python
'''
Created on 13.04.2015

@author: stephan
'''
import rospy
import random # because why not?
from buildingBlocks.conditions import Condition, Disjunction
from buildingBlocks.sensors import Sensor
from buildingBlocks.behaviour import Behaviour
from buildingBlocks.activators import BooleanActivator, LinearActivator
from buildingBlocks.goals import Goal
from buildingBlocks.managers import Manager

if __name__ == '__main__':
    rospy.init_node('behaviourPlanner', anonymous=True) 
    # some random helper variables
    homeSelected = False
    # create a Manager
    m = Manager()
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
    fullBattery = Condition(batterySensor, LinearActivator(.05, .3), name = "fullBatteryCondition")
    emptyBattery = Condition(batterySensor, LinearActivator(.5, .1), name = "emptyBatteryCondition")
    isFlying = Condition(flyingSensor, BooleanActivator(True), name = "isFlyingCondition")
    isNotFlying = Condition(flyingSensor, BooleanActivator(False), name = "isNotFlyingCondition")
    isAtHome = Condition(homeSensor, BooleanActivator(True), name = "isAtHomeCondition")
    isNotAtHome = Condition(homeSensor, BooleanActivator(False), name = "isNotAtHomeCondition")
    targetSelected = Condition(targetSelectedSensor, BooleanActivator(True), name = "targetSelectedCondition")
    targetNotSelected = Condition(targetSelectedSensor, BooleanActivator(False), name = "targetNotSelectedCondition")
    objectsFound = Condition(objectsFoundSensor, LinearActivator(0, 1), name = "objectsFoundCondition")
    objectsNotFound = Condition(objectsFoundSensor, LinearActivator(1, 0), name = "objectsNotFoundCondition")
    mapComplete = Condition(mapCoverageSensor, LinearActivator(0, 1), name = "mapCompleteCondition")
    mapIncomplete = Condition(mapCoverageSensor, LinearActivator(1, 0), name = "mapIncompleteCondition")
    # setting up behaviours
    startBehaviour = m.addBehaviour(Behaviour("startBehaviour", correlations = {flyingSensor: 1.0}))
    def startAction():
        flyingSensor.update(True)
        return False
    startBehaviour.action = startAction
    startBehaviour.addPrecondition(isNotFlying)
    startBehaviour.addPrecondition(fullBattery)
    landBehaviour = m.addBehaviour(Behaviour("landBehaviour", correlations = {flyingSensor: -1.0}))
    def landAction():
        flyingSensor.update(False)
        return False
    landBehaviour.action = landAction
    landBehaviour.addPrecondition(isFlying)
    landBehaviour.addPrecondition(emptyBattery)
    landBehaviour.addPrecondition(isAtHome)
    goHomeBehaviour = m.addBehaviour(Behaviour("goHomeBehaviour", correlations = {targetSelectedSensor: 1.0}))
    def goHomeAction():
        targetSelectedSensor.update(True)
        global homeSelected
        homeSelected = True
        return False
    goHomeBehaviour.action = goHomeAction
    goHomeBehaviour.addPrecondition(isNotAtHome)
    goHomeBehaviour.addPrecondition(emptyBattery)
    selectTargetBehaviour = m.addBehaviour(Behaviour("selectTargetBehaviour", correlations = {targetSelectedSensor: 1.0}))
    def selectTargetActionAction():
        targetSelectedSensor.update(True)
        return False
    selectTargetBehaviour.action = selectTargetActionAction
    selectTargetBehaviour.readyThreshold = 0.42
    selectTargetBehaviour.addPrecondition(fullBattery)
    selectTargetBehaviour.addPrecondition(isFlying)
    selectTargetBehaviour.addPrecondition(Disjunction(objectsNotFound, mapIncomplete, name = "noMapNorObjectsDisjunction"))
    moveBehaviour = m.addBehaviour(Behaviour("moveBehaviour", correlations = {homeSensor: 0.8, mapCoverageSensor: 0.8, objectsFoundSensor: 0.8, targetSelectedSensor: -0.5}))
    def moveAction():
        homeSensor.update(homeSelected)
        mapImprovement = 1337
        for i in range(10):
            if mapImprovement + mapCoverageSensor.value <= 1:
                mapCoverageSensor.update(mapImprovement + mapCoverageSensor.value / 2)
            else:
                mapImprovement = random.random()        
        objectImprovement = 1337
        for i in range(10):
            if objectImprovement + objectsFoundSensor.value <= 1:
                objectsFoundSensor.update(objectImprovement + objectsFoundSensor.value / 2)
            else:
                objectImprovement = random.random()
        targetSelectedSensor.update(False)
        return False
    moveBehaviour.action = moveAction        
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
    rate = rospy.Rate(2) # 2hz
    for i in range(25):
        m.step()
        batterySensor.update(batterySensor.value - 0.05)
        rate.sleep()