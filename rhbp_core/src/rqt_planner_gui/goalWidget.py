## rqt widget displaying a goal
#Created on 10.08.2015
#@author: stephan

import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from behaviour_planner.srv import Activate, SetInteger
from PyQt4.QtCore import pyqtSignal

# Custum Widget for goal
class GoalWidget(QWidget):
    updateGUIsignal = pyqtSignal(dict)
    def __init__(self, name):
        super(GoalWidget, self).__init__()
        self._name = name
        # Give QObjects reasonable names
        self.setObjectName(self._name + 'Widget')
        # Get path to UI file which should be in the "resource" folder of this node
        ui_file = os.path.join(rospkg.RosPack().get_path('behaviour_planner'), 'src', 'rqt_planner_gui', 'resource', 'goal.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.goalGroupBox.setTitle(self._name)
        self.activatedCheckbox.toggled.connect(self.activationCallback)
        self.priorityButton.clicked.connect(self.setPriorityCallback)
        self.updateGUIsignal.connect(self.updateGUI)

    def __del__(self):
        self.__deleted = True
    
    def updateGUI(self, newValues):
        self.activatedCheckbox.setChecked(newValues["activated"])
        self.fulfillmentDoubleSpinBox.setValue(newValues["fulfillment"])
        self.fulfillmentDoubleSpinBox.setToolTip("{0}".format(newValues["fulfillment"]))
        self.activeLabel.setText(newValues["active"])
        self.wishesLabel.setText(newValues["wishes"])
        self.wishesLabel.setToolTip(newValues["wishesTooltip"])
        if not self.prioritySpinBox.hasFocus():
            self.prioritySpinBox.setValue(newValues["priority"])
        
    def refresh(self, msg):
        """
        Refreshes the widget with data from the new message.
        """
        assert self._name == msg.name
        self.updateGUIsignal.emit({
                                   "activated" : msg.activated,
                                   "fulfillment" : msg.satisfaction,
                                   "active" : str(msg.active),
                                   "priority" : msg.priority,
                                   "wishes" : "\n".join(map(lambda x: "{0}: {1:.4g}".format(x.sensorName, x.indicator), msg.wishes)),
                                   "wishesTooltip" : "\n".join(map(lambda x: "{0}: {1}".format(x.sensorName, x.indicator), msg.wishes))
                                  })
    
    def activationCallback(self, status):
        rospy.logdebug("Waiting for service %s", self._name + 'Activate')
        rospy.wait_for_service(self._name + 'Activate')
        activateRequest = rospy.ServiceProxy(self._name + 'Activate', Activate)
        activateRequest(status)
        rospy.logdebug("Set activated of %s goal to %s", self._name, status)
       
    def setPriorityCallback(self):
        rospy.logdebug("Waiting for service %s", self._name + 'Priority')
        rospy.wait_for_service(self._name + 'Priority')
        priorityRequest = rospy.ServiceProxy(self._name + 'Priority', SetInteger)
        priorityRequest(self.prioritySpinBox.value())
        rospy.logdebug("Set priority of %s to %s", self._name, self.prioritySpinBox.value())
