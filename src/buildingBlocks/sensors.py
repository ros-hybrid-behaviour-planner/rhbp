'''
Created on 13.04.2015

@author: stephan
'''

class sensor(object):
    '''
    This class represents information necessary to make decisions.
    Although it is not abstract it will be barely useful and should be used as base class for actual clever implementations.
    
    '''
    _value = None # this is what it's all about. Of course, the type and how it is acquired will change depending on the specific sensor

    def __init__(self):
        '''
        Constructor
        '''
    
    def getValue(self):
        return self._value
    
    def update(self, newValue):
        '''
        This method is to refresh the _value. It may serve as a callback for ROS subscriptions or be called from some kind of main loop.
        '''
        self._value = newValue