'''
Created on 13.04.2015

@author: stephan
'''
#from buildingBlocks.activators import thresholdActivator
import buildingBlocks.activators
from buildingBlocks.sensors import Sensor
from buildingBlocks.behaviour import Behaviour


if __name__ == '__main__':
    s1 = Sensor()
    a1 = buildingBlocks.activators.ThresholdActivator(s1, 90, 0)
    s1.update(10)
    s2 = Sensor()
    s2.update(90)
    a2 = buildingBlocks.activators.ThresholdActivator(s2, 0, 100)
    b = Behaviour()
    b.addActivator(a1)
    b.addActivator(a2)
    print b, s1, s2, a1, a2
    print b.getActivation()