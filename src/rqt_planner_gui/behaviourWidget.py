## rqt widget displaying a behaviour
#Created on 10.08.2015
#@author: stephan

import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from behaviourPlannerPython.srv import Activate, ForceStart, Priority

# Custum Widget for Behaviour
class BehaviourWidget(QWidget):
    def __init__(self, name, plugin):
        super(BehaviourWidget, self).__init__()
        self._name = name
        self._overviewPlugin = plugin # this is where we get the planner prefix from
        # Give QObjects reasonable names
        self.setObjectName(self._name + 'Widget')
        # Get path to UI file which should be in the "resource" folder of this node
        ui_file = os.path.join(rospkg.RosPack().get_path('behaviourPlannerPython'), 'src', 'rqt_planner_gui', 'resource', 'behaviour.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.behaviourGroupBox.setTitle(self._name)
        self.activatedCheckbox.toggled.connect(self.activationCallback)
        self.forceStartCheckbox.toggled.connect(self.forceStartCallback)
        self.priorityPushButton.clicked.connect(self.setPriorityCallback)

    def __del__(self):
        self.__deleted = True
        
    def refresh(self, msg):
        """
        Refreshes the widget with data from the new message.
        """
        assert self._name == msg.name
        self.activatedCheckbox.setChecked(msg.activated)
        self.satisfactionDoubleSpinBox.setValue(msg.satisfaction)
        self.activeCheckbox.setChecked(msg.active)
        self.wishesLabel.setText("\n".join(map(lambda x: "{0}: {1:.4g}".format(x.sensorName, x.indicator), msg.wishes)))
        self.wishesLabel.setToolTip("\n".join(map(lambda x: "{0}: {1}".format(x.sensorName, x.indicator), msg.wishes)))
        self.correlationsLabel.setText("\n".join(map(lambda x: "{0}: {1:.4g}".format(x.sensorName, x.indicator), msg.correlations)))
        self.correlationsLabel.setToolTip("\n".join(map(lambda x: "{0}: {1}".format(x.sensorName, x.indicator), msg.correlations)))
        self.activationDoubleSpinBox.setValue(msg.activation)
        self.activationDoubleSpinBox.setToolTip("{0}".format(msg.activation))
        self.satisfactionDoubleSpinBox.setValue(msg.satisfaction)
        self.satisfactionDoubleSpinBox.setToolTip("{0}".format(msg.satisfaction))
        self.readyThresholdDoubleSpinBox.setValue(msg.threshold)
        self.readyThresholdDoubleSpinBox.setToolTip("{0}".format(msg.threshold))
        self.executableCheckbox.setChecked(msg.executable)
        self.isExecutingCheckbox.setChecked(msg.isExecuting)
        self.progressDoubleSpinBox.setValue(msg.progress)
        self.progressDoubleSpinBox.setToolTip("{0}".format(msg.progress))
        if not self.prioritySpinBox.hasFocus():
            self.prioritySpinBox.setValue(msg.priority)
        self.interruptableCheckbox.setChecked(msg.interruptable)
    
    def activationCallback(self, status):
        rospy.logdebug("Waiting for service %s", self._name + 'Activate')
        rospy.wait_for_service(self._name + 'Activate')
        activateRequest = rospy.ServiceProxy(self._name + 'Activate', Activate)
        activateRequest(status)
        rospy.logdebug("Set activated of %s to %s", self._name, status)
        
    def forceStartCallback(self, status):
        rospy.logdebug("Waiting for service %s", self._overviewPlugin.plannerPrefix + 'ForceStart')
        rospy.wait_for_service(self._overviewPlugin.plannerPrefix + 'ForceStart')
        activateRequest = rospy.ServiceProxy(self._overviewPlugin.plannerPrefix + 'ForceStart', ForceStart)
        activateRequest(self._name, status)
        rospy.logdebug("Set forceStart of %s to %s", self._name, status)
    
    def setPriorityCallback(self):
        rospy.logdebug("Waiting for service %s", self._name + 'Priority')
        rospy.wait_for_service(self._name + 'Priority')
        priorityRequest = rospy.ServiceProxy(self._name + 'Priority', Priority)
        priorityRequest(self.prioritySpinBox.value())
        rospy.logdebug("Set priority of %s to %s", self._name, self.prioritySpinBox.value())
        