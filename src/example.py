'''
Created on 13.04.2015

@author: stephan
'''

from buildingBlocks.conditions import Condition, Disjunction
from buildingBlocks.sensors import Sensor
from buildingBlocks.behaviour import Behaviour
from buildingBlocks.activators import BooleanActivator, LinearActivator
from buildingBlocks.goals import Goal
from buildingBlocks.managers import Manager

if __name__ == '__main__':  
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
    fullBattery = Condition(batterySensor, LinearActivator(.05, 1), name = "fullBatteryCondition")
    emptyBattery = Condition(batterySensor, LinearActivator(1, .1), name = "emptyBatteryCondition")
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
    startBehaviour = m.addBehaviour(Behaviour("startBehaviour"))
    startBehaviour.addPrecondition(isNotFlying)
    startBehaviour.addPrecondition(fullBattery)
    landBehaviour = m.addBehaviour(Behaviour("landBehaviour"))
    landBehaviour.addPrecondition(isFlying)
    landBehaviour.addPrecondition(emptyBattery)
    goHomeBehaviour = m.addBehaviour(Behaviour("goHomeBehaviour"))
    goHomeBehaviour.addPrecondition(isNotAtHome)
    goHomeBehaviour.addPrecondition(emptyBattery)
    selectTargetBehaviour = m.addBehaviour(Behaviour("selectTargetBehaviour"))
    selectTargetBehaviour.addPrecondition(fullBattery)
    selectTargetBehaviour.addPrecondition(isFlying)
    selectTargetBehaviour.addPrecondition(Disjunction(objectsNotFound, mapIncomplete, name = "noMapNorObjectsDisjunction"))
    moveBehaviour = m.addBehaviour(Behaviour("moveBehaviour"))
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
    for behaviour in m.behaviours:
        print behaviour
        print "executable: {0} ({1})".format(behaviour.executable, behaviour.getPreconditionSatisfaction())
        print behaviour.name, "wishes", behaviour.getWishes()
        print "activation from preconditions: ", behaviour.getActivationFromPreconditions()
        print
    for goal in m.goals:
        print goal.name, "satisfaction", goal.statisfaction, "wishes", goal.getWishes()