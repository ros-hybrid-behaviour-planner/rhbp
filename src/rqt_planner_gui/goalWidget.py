## rqt widget displaying a goal
#Created on 10.08.2015
#@author: stephan

import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from behaviourPlannerPython.srv import Activate

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
        self.activatedCheckbox.toggled.connect(self.activationCallback)

    def __del__(self):
        self.__deleted = True
        
    def refresh(self, msg):
        """
        Refreshes the widget with data from the new message.
        """
        assert self._name == msg.name
        self.activatedCheckbox.setChecked(msg.activated)
        self.fulfillmentDoubleSpinBox.setValue(msg.satisfaction)
        self.activeCheckbox.setChecked(msg.active)
        self.wishesLabel.setText("\n".join(map(lambda x: "{0}: {1:.4g}".format(x.sensorName, x.indicator), msg.wishes)))
        self.wishesLabel.setToolTip("\n".join(map(lambda x: "{0}: {1}".format(x.sensorName, x.indicator), msg.wishes)))
    
    def activationCallback(self, status):
        rospy.logdebug("Waiting for service %s", self._name + 'Activate')
        rospy.wait_for_service(self._name + 'Activate')
        activateRequest = rospy.ServiceProxy(self._name + 'Activate', Activate)
        activateRequest(status)
        rospy.logdebug("Set activated of %s goal to %s", self._name, status)
       