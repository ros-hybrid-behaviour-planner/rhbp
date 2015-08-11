## rqt widget displaying a goal
#Created on 10.08.2015
#@author: stephan

import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

# Custum Widget for goal
class GoalWidget(QWidget):
    def __init__(self, name):
        super(GoalWidget, self).__init__()
        self._name = name
        # Give QObjects reasonable names
        self.setObjectName(self._name + 'Widget')
        # Get path to UI file which should be in the "resource" folder of this node
        ui_file = os.path.join(rospkg.RosPack().get_path('behaviourPlannerPython'), 'src', 'rqt_planner_gui', 'resource', 'goal.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.goalGroupBox.setTitle(self._name)

    def __del__(self):
        self.__deleted = True
        
    def refresh(self, msg):
        """
        Refreshes the widget with data from the new message.
        """
        assert self._name == msg.name
        rospy.logdebug("refreshing %s with fulfillment %f", self._name, msg.satisfaction)
       