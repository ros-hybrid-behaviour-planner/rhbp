## rqt widget displaying a behaviour
#Created on 10.08.2015
#@author: stephan wypler

import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from rhbp_core.srv import Enable, ForceStart, SetInteger
from rhbp_core.msg import Status
from PyQt5.QtCore import pyqtSignal

# Custum Widget for Behaviour
class BehaviourWidget(QWidget):
    updateGUIsignal = pyqtSignal(dict)
    
    def __init__(self, name, plugin):
        super(BehaviourWidget, self).__init__()
        self._name = name
        self._overviewPlugin = plugin # this is where we get the planner prefix from
        # Give QObjects reasonable names
        self.setObjectName(self._name + 'Widget')
        # Get path to UI file which should be in the "resource" folder of this node
        ui_file = os.path.join(rospkg.RosPack().get_path('rhbp_core'), 'src', 'rqt_planner_gui', 'resource', 'behaviour.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.behaviourGroupBox.setTitle(self._name)
        self.activatedCheckbox.toggled.connect(self.activationCallback)
        self.forceStartButton.toggled.connect(self.forceStartCallback)
        self.priorityPushButton.clicked.connect(self.setPriorityCallback)
        self.executionTimeoutPushButton.clicked.connect(self.setExecutionTimeoutCallback)
        self.updateGUIsignal.connect(self.updateGUI)

    def __del__(self):
        self.__deleted = True
    
    def updateGUI(self, newValues):
        self.activatedCheckbox.setChecked(newValues["enabled"])
        self.satisfactionDoubleSpinBox.setValue(newValues["satisfaction"])
        self.satisfactionDoubleSpinBox.setToolTip("{0}".format(newValues["satisfaction"]))
        self.activeLabel.setText(newValues["active"])
        self.wishesLabel.setText(newValues["wishes"])
        self.wishesLabel.setToolTip(newValues["wishesTooltip"])
        self.correlationsLabel.setText(newValues["correlations"])
        self.correlationsLabel.setToolTip(newValues["correlationsTooltip"])
        self.activationDoubleSpinBox.setValue(newValues["activation"])
        self.activationDoubleSpinBox.setToolTip("{0}".format(newValues["activationTooltip"]))
        self.readyThresholdDoubleSpinBox.setValue(newValues["readyThreshold"])
        self.readyThresholdDoubleSpinBox.setToolTip("{0}".format(newValues["readyThreshold"]))
        self.executableLabel.setText(newValues["executable"])
        self.isExecutingLabel.setText(newValues["isExecuting"])
        self.executionTimeSpinBox.setValue(newValues["executionTime"])
        if not self.prioritySpinBox.hasFocus():
            self.prioritySpinBox.setValue(newValues["priority"])
        if not self.executionTimeoutSpinBox.hasFocus():
            self.executionTimeoutSpinBox.setValue(newValues["executionTimeout"])
        self.interruptableLabel.setText(newValues["interruptable"])
        self.independentFromPlannerLabel.setText(newValues["independentFromPlanner"])
        
    def refresh(self, msg):
        """
        Refreshes the widget with data from the new message.
        :type msg: Status
        """
        assert self._name == msg.name
        self.updateGUIsignal.emit({
                                   "enabled" : msg.enabled,
                                   "satisfaction" : msg.satisfaction,
                                   "active" : str(msg.active),
                                   #TODO information could be extended here
                                   "wishes" : "\n".join(map(lambda x: "{0}: {1:.4g}".format(x.sensorName, x.indicator), msg.wishes)),
                                   "wishesTooltip" : "\n".join(map(lambda x: "{0}: {1}".format(x.sensorName, x.indicator), msg.wishes)),
                                   #TODO information could be extended here
                                   "correlations" : "\n".join(map(lambda x: "{0}: {1:.4g}".format(x.sensorName, x.indicator), msg.correlations)),
                                   "correlationsTooltip" : "\n".join(map(lambda x: "{0}: {1}".format(x.sensorName, x.indicator), msg.correlations)),
                                   "activation" : msg.activation,
                                   "activationTooltip": "\n".join(map(lambda x: "{0}: {1}".format(x.name, x.activation), msg.activations)),
                                   "readyThreshold" : msg.threshold,
                                   "executable" : str(msg.executable),
                                   "isExecuting" : str(msg.isExecuting),
                                   "executionTimeout" : msg.executionTimeout,
                                   "executionTime" : msg.executionTime,
                                   "priority" : msg.priority,
                                   "interruptable" : str(msg.interruptable),
                                   "independentFromPlanner": str(msg.independentFromPlanner)
                                  })

    def _get_service_prefix(self):
        """
        generate the service prefix based on the current planner prefix value
        :return: str prefix
        """
        return self._overviewPlugin.planner_prefix + '/' + self._name + '/'
    
    def activationCallback(self, status):
        service_name = self._get_service_prefix() + 'Enable'
        rospy.logdebug("Waiting for service %s", service_name)
        rospy.wait_for_service(service_name)
        activateRequest = rospy.ServiceProxy(service_name, Enable)
        activateRequest(status)
        rospy.logdebug("Set enabled of %s to %s", self._name, status)
        
    def forceStartCallback(self, status):
        service_name = self._overviewPlugin.planner_prefix + '/'+ 'ForceStart'
        rospy.logdebug("Waiting for service %s", )
        rospy.wait_for_service(service_name)
        forceStartRequest = rospy.ServiceProxy(service_name, ForceStart)
        forceStartRequest(self._name, status)
        if status == True:
            self.forceStartButton.setText("back to normal")
        else:
            self.forceStartButton.setText("start")
        rospy.logdebug("Set forceStart of %s to %s", self._name, status)
    
    def setPriorityCallback(self):
        service_name = self._get_service_prefix() + 'Priority'
        rospy.logdebug("Waiting for service %s", service_name)
        rospy.wait_for_service(service_name)
        priorityRequest = rospy.ServiceProxy(service_name, SetInteger)
        priorityRequest(self.prioritySpinBox.value())
        rospy.logdebug("Set priority of %s to %s", self._name, self.prioritySpinBox.value())
    
    def setExecutionTimeoutCallback(self):
        service_name = self._get_service_prefix() + 'ExecutionTimeout'
        rospy.logdebug("Waiting for service %s", service_name)
        rospy.wait_for_service(service_name)
        priorityRequest = rospy.ServiceProxy(service_name, SetInteger)
        priorityRequest(self.executionTimeoutSpinBox.value())
        rospy.logdebug("Set executionTimeout of %s to %s", self._name, self.executionTimeoutSpinBox.value())


