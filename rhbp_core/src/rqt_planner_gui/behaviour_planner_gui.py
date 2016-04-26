## rqt widget showing global behaviour planner attributes and providing a container for behaviour widgets
#Created on 10.08.2015
#@author: stephan wypler

import os
import rospy
import rospkg
from std_srvs.srv import Empty
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from behaviourWidget import BehaviourWidget
from goalWidget import GoalWidget
from rhbp_core.msg import PlannerStatus
from PyQt4.QtCore import pyqtSignal


class Overview(Plugin):
    updateRequest = pyqtSignal(dict)
    addBehaviourRequest = pyqtSignal(str)
    addGoalRequest = pyqtSignal(str)

    def __init__(self, context):
        super(Overview, self).__init__(context)
        
        self.__behaviours = {} # this stores all behaviours
        self.__goals = {}      # this stores all goals
        self.__plannerPrefix = ""
        
        # Give QObjects reasonable names
        self.setObjectName('behaviour_planner_overview')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-p", "--plannerPrefix", dest="plannerPrefix", help="Specify the planner prefix")
        args, unknowns = parser.parse_known_args(context.argv())
        if args.plannerPrefix:
            self.__plannerPrefix = args.plannerPrefix
            rospy.loginfo("using planner prefix %s", self.__plannerPrefix)
        
        # subscribe to our information source
        self.__sub = rospy.Subscriber("/" + self.__plannerPrefix + "Planner/plannerStatus", PlannerStatus, self.plannerStatusCallback)
            
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this node
        ui_file = os.path.join(rospkg.RosPack().get_path('rhbp_core'), 'src', 'rqt_planner_gui', 'resource', 'overview.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('OverviewUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.activationThresholdDecayPushButton.clicked.connect(self.setActivationThresholdDecay)
        self._widget.plannerPrefixPushButton.clicked.connect(self.setPlannerPrefix)
        self._widget.pausePushButton.toggled.connect(self.pauseButtonCallback)

        # Connect signal so we can refresh Widgets from the main thread        
        self.updateRequest.connect(self.updateGUI)
        self.addBehaviourRequest.connect(self.addBehaviourWidget)
        self.addGoalRequest.connect(self.addGoalWidget)
        
    def updateGUI(self, newValues):
        self._widget.activationThresholdDoubleSpinBox.setValue(newValues["activationThreshold"])
        if not self._widget.activationThresholdDecayDoubleSpinBox.hasFocus():
            self._widget.activationThresholdDecayDoubleSpinBox.setValue(newValues["activationThresholdDecay"])
        self._widget.influencedSensorsLabel.setText(newValues["influencedSensors"])
        self._widget.runningBehavioursLabel.setText(newValues["runningBehaviours"])
    
    def addBehaviourWidget(self, name):
        self.__behaviours[name] = BehaviourWidget(name, self)
        self._widget.behaviourFrame.layout().addWidget(self.__behaviours[name])
        
    def addGoalWidget(self, name):
        self.__goals[name] = GoalWidget(name)
        self._widget.goalFrame.layout().addWidget(self.__goals[name])
    
    def pauseButtonCallback(self, status):
        try:
            if status == True:
                rospy.logdebug("Waiting for service %s", self.__plannerPrefix + 'Pause')
                rospy.wait_for_service(self.__plannerPrefix + 'Pause')
                pauseRequest = rospy.ServiceProxy(self.__plannerPrefix + 'Pause', Empty)
                pauseRequest()
                self._widget.pausePushButton.setText("resume")
            else:
                rospy.logdebug("Waiting for service %s", self.__plannerPrefix + 'Resume')
                rospy.wait_for_service(self.__plannerPrefix + 'Resume')
                resumeRequest = rospy.ServiceProxy(self.__plannerPrefix + 'Resume', Empty)
                resumeRequest()
                self._widget.pausePushButton.setText("pause")
        except Exception as e:
            rospy.logerr("error while toggling pause or resume: %s", str(e))
    
    def updateBehaviour(self, msg):
        """
        Add the behaviour to our management structure if it does not exist in there.
        Update the behaviour Widget using its refresh() method.
        """
        if not msg.name in self.__behaviours:
            self.addBehaviourRequest.emit(msg.name)
        try:
            self.__behaviours[msg.name].refresh(msg)
        except KeyError as e: # this happens at the first time because the GUI thread with the main loop did not create the widget yet
            rospy.logdebug("%s", e)
    
    def updateGoal(self, msg):
        """
        Add the goal to our management structure if it does not exist in there.
        Update the goal Widget using its refresh() method.
        """
        if not msg.name in self.__goals:
            self.addGoalRequest.emit(msg.name)
        try:
            self.__goals[msg.name].refresh(msg)
        except KeyError as e: # this happens at the first time because the GUI thread with the main loop did not create the widget yet
            rospy.logdebug("%s", e)
    
    def setActivationThresholdDecay(self):
        rospy.loginfo("setting activationThresholdDecay to %f", self._widget.activationThresholdDecayDoubleSpinBox.value())
        rospy.set_param("activationThresholdDecay", self._widget.activationThresholdDecayDoubleSpinBox.value())
    
    def setPlannerPrefix(self):
        self.__sub.unregister()
        self.__plannerPrefix = self._widget.plannerPrefixEdit.text()
        rospy.loginfo("subscribing to %s", "/" + self.__plannerPrefix + "Planner/plannerStatus")
        self.__sub = rospy.Subscriber("/" + self.__plannerPrefix + "Planner/plannerStatus", PlannerStatus, self.plannerStatusCallback)
    
    def plannerStatusCallback(self, msg):
        rospy.logdebug("received %s", msg)
        try:
            for behaviour in msg.behaviours:
                self.updateBehaviour(behaviour)
            for goal in msg.goals:
                self.updateGoal(goal)
            self.updateRequest.emit({
                                     "activationThreshold" : msg.activationThreshold,
                                     "activationThresholdDecay" : msg.activationThresholdDecay,
                                     "influencedSensors" : ", ".join(msg.influencedSensors),
                                     "runningBehaviours" : ", ".join(msg.runningBehaviours)
                                    }) 
        except Exception as e:
            rospy.logerr("%s", e)
    
    @property
    def plannerPrefix(self):
        return self.__plannerPrefix
        
    def shutdown_plugin(self):
        self.__sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        rospy.loginfo("saving plannerPrefix setting")
        instance_settings.set_value("plannerPrefix", self.__plannerPrefix)

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        rospy.loginfo("restoring plannerPrefix setting")
        storedPlannerPrefix = instance_settings.value("plannerPrefix")
        if type(storedPlannerPrefix) == unicode:
            storedPlannerPrefix = storedPlannerPrefix.encode('ascii','ignore')
        if storedPlannerPrefix:
            self._widget.plannerPrefixEdit.setText(storedPlannerPrefix)
            self.setPlannerPrefix()

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
