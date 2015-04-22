'''
Created on 13.04.2015

@author: stephan
'''

from buildingBlocks.conditions import Condition
from buildingBlocks.sensors import Sensor
from buildingBlocks.behaviour import Behaviour
from buildingBlocks.activators import BooleanActivator, ThresholdActivator, LinearActivator
from buildingBlocks.goals import Goal


if __name__ == '__main__':
    s1 = Sensor()
    p1 = Condition(s1, LinearActivator(90, 0))
    s1.update(10)
    s2 = Sensor()
    s2.update(90)
    p2 = Condition(s2, LinearActivator(0, 100))
    p3 = Condition(s2, ThresholdActivator(90, True, maxActivation = 0.9))
    s3 = Sensor()
    s3.update(True)
    p4 = Condition(s3, BooleanActivator(True))
    p4.activator.minActivation = .1
    b = Behaviour()
    b.preconditions.append(p1) # not recommended as there is no type checking that you really append a precondition!
    b.addPrecondition(p2) # recommended because of internal type checking
    for x in (b, s1, s2, s3, p1, p2, p3, p4):
        print x
    print b, "precondition activation:", b._getPreconditionActivation()
    print b, "readyThreshold:", b.readyThreshold
    print b, "ready:", b.executable
    print p3.activation
    print p4.activation
    print b.preconditions
    g1 = Goal()
    g1.addCondition(p3)
    g1.addCondition(p4)
    print g1
    print "satisfaction:", g1.statisfaction