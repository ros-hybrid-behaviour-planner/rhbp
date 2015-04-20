'''
Created on 13.04.2015

@author: stephan
'''
#from buildingBlocks.activators import thresholdActivator
import buildingBlocks.activators
from buildingBlocks.sensors import sensor
from buildingBlocks.behaviour import behaviour


if __name__ == '__main__':
    s1 = sensor()
    a1 = buildingBlocks.activators.thresholdActivator(s1, 90, 0)
    s1.update(10)
    s2 = sensor()
    s2.update(90)
    a2 = buildingBlocks.activators.thresholdActivator(s2, 0, 100)
    b = behaviour("behaviour")
    b.addDependency(a1)
    b.addDependency(a2)
    print b.getActivation()