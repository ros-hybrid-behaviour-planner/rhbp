'''
Created on 20.04.2015

@author: stephan
'''

class Actuator(object):
    '''
    This is the base class for an actuator. Extend it to let it publish to ROS or anything.
    '''
    
    _instanceCounter = 0

    def __init__(self, name = None):
        '''
        Constructor
        '''
        self._name = name if name else "Actuator {0}".format(Actuator._instanceCounter)
        Actuator._instanceCounter += 1
    
    def __str__(self):
        return self._name