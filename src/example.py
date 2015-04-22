'''
Created on 13.04.2015

@author: stephan
'''

from buildingBlocks.conditions import Condition, Disjunction
from buildingBlocks.sensors import Sensor
from buildingBlocks.behaviour import Behaviour
from buildingBlocks.activators import BooleanActivator, LinearActivator
from buildingBlocks.goals import Goal


if __name__ == '__main__':
    # creating sensors
    batterySensor = Sensor("batteryLevelSensor")
    flyingSensor = Sensor("isFlyingSensor")
    homeSensor = Sensor("homeSensor")
    targetSelectedSensor = Sensor("targetSelectedSensor")
    objectsFoundSensor = Sensor("objectsFoundSensor")
    mapCoverageSensor = Sensor("mapCoverageSensor")
    # initial conditions
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
    startBehaviour = Behaviour("startBehaviour")
    startBehaviour.addPrecondition(isNotFlying)
    startBehaviour.addPrecondition(fullBattery)
    landBehaviour = Behaviour("landBehaviour")
    landBehaviour.addPrecondition(isFlying)
    landBehaviour.addPrecondition(emptyBattery)
    goHomeBehaviour = Behaviour("goHomeBehaviour")
    goHomeBehaviour.addPrecondition(isNotAtHome)
    goHomeBehaviour.addPrecondition(emptyBattery)
    selectTargetBehaviour = Behaviour("selectTargetBehaviour")
    selectTargetBehaviour.addPrecondition(fullBattery)
    selectTargetBehaviour.addPrecondition(Disjunction(objectsNotFound, mapIncomplete, name = "noMapNorObjectsDisjunction"))
    moveBehaviour = Behaviour("moveBehaviour")
    moveBehaviour.addPrecondition(isFlying)
    moveBehaviour.addPrecondition(targetSelected)
    # setting up goals
    returnHomeGoal = Goal("returnedHome")
    returnHomeGoal.addCondition(isNotFlying)
    returnHomeGoal.addCondition(isAtHome)
    completeMapGoal = Goal("completedMap")
    completeMapGoal.addCondition(mapComplete)
    objectsFoundGoal = Goal("objectsFound")
    objectsFoundGoal.addCondition(objectsFound)
    print startBehaviour
    print "executable: {0} ({1})".format(startBehaviour.executable, startBehaviour._getPreconditionActivation())
    print
    print landBehaviour
    print "executable: {0} ({1})".format(landBehaviour.executable, landBehaviour._getPreconditionActivation())
    print
    print goHomeBehaviour
    print "executable: {0} ({1})".format(goHomeBehaviour.executable, goHomeBehaviour._getPreconditionActivation())
    print
    print selectTargetBehaviour
    print "executable: {0} ({1})".format(selectTargetBehaviour.executable, selectTargetBehaviour._getPreconditionActivation())
    print
    print moveBehaviour
    print "executable: {0} ({1})".format(moveBehaviour.executable, moveBehaviour._getPreconditionActivation())
