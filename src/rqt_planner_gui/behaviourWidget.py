## rqt widget displaying a behaviour
#Created on 10.08.2015
#@author: stephan

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import pyqtSignal


# Custum Widget for Behaviour
class BehaviourWidget(QWidget):
    updated = pyqtSignal() # this is a class variable but for some reason it does not work with an instance variable

    def __init__(self, name):
        super(BehaviourWidget, self).__init__()
        self._name = name
        
        # Give QObjects reasonable names
        self.setObjectName(self._name + 'Widget')
                
        # Get path to UI file which should be in the "resource" folder of this node
        ui_file = os.path.join(rospkg.RosPack().get_path('behaviourPlannerPython'), 'src', 'rqt_planner_gui', 'resource', 'behaviour.ui')
        
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        
        self.behaviourGroupBox.setTitle(self._name)

    def __del__(self):
        self.__deleted = True
        
    def refresh(self):
        """
        Refreshes the widget.
        """
        print "refresh"
        
    def onUpdate(self):
        """
        Updates the widget.
        """
        print "onUpdate"
        self.updated.emit()
       