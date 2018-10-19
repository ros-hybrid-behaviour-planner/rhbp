## rqt widget displaying a goal
#Created on 10.08.2015
#@author: stephan

import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from rhbp_core.srv import Enable, SetInteger
from PyQt5.QtCore import pyqtSignal

# Custum Widget for goal
class GoalWidget(QWidget):
    updateGUIsignal = pyqtSignal(dict)
    def __init__(self, name, plugin):
        super(GoalWidget, self).__init__()
        self._name = name
        self._overviewPlugin = plugin # this is where we get the planner prefix from
        # Give QObjects reasonable names
        self.setObjectName(self._name + 'Widget')
        # Get path to UI file which should be in the "resource" folder of this node
        ui_file = os.path.join(rospkg.RosPack().get_path('rhbp_core'), 'src', 'rqt_planner_gui', 'resource', 'goal.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.goalGroupBox.setTitle(self._name)
        self.activatedCheckbox.toggled.connect(self.activationCallback)
        self.priorityButton.clicked.connect(self.setPriorityCallback)
        self.updateGUIsignal.connect(self.updateGUI)

    def _get_service_prefix(self):
        """
        generate the service prefix based on the current planner prefix value
        :return: str prefix
        """
        return self._overviewPlugin.planner_prefix + '/' + self._name + '/'

    def __del__(self):
        self.__deleted = True
    
    def updateGUI(self, newValues):
        self.activatedCheckbox.setChecked(newValues["enabled"])
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
                                   "enabled" : msg.enabled,
                                   "fulfillment" : msg.satisfaction,
                                   "active" : str(msg.active),
                                   "priority" : msg.priority,
                                   "wishes" : "\n".join(map(lambda x: "{0}: {1:.4g}".format(x.sensorName, x.indicator), msg.wishes)),
                                   "wishesTooltip" : "\n".join(map(lambda x: "{0}: {1}".format(x.sensorName, x.indicator), msg.wishes))
                                  })
    
    def activationCallback(self, status):
        service_name = self._get_service_prefix() + 'Enable'
        rospy.logdebug("Waiting for service %s", service_name)
        rospy.wait_for_service(service_name)
        activateRequest = rospy.ServiceProxy(service_name, Enable)
        activateRequest(status)
        rospy.logdebug("Set enabled of %s goal to %s", self._name, status)
       
    def setPriorityCallback(self):
        service_name = self._get_service_prefix() + 'Priority'
        rospy.logdebug("Waiting for service %s", service_name)
        rospy.wait_for_service(service_name)
        priorityRequest = rospy.ServiceProxy(service_name, SetInteger)
        priorityRequest(self.prioritySpinBox.value())
        rospy.logdebug("Set priority of %s to %s", self._name, self.prioritySpinBox.value())
