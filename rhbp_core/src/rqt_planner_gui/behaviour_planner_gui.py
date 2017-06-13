## rqt widget showing global behaviour planner attributes and providing a container for behaviour widgets
#Created on 10.08.2015
#@author: stephan wypler

import os
import rospy
import rospkg
from std_srvs.srv import Empty
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from behaviourWidget import BehaviourWidget
from goalWidget import GoalWidget
from rhbp_core.msg import PlannerStatus
from PyQt5.QtCore import pyqtSignal


class Overview(Plugin):
    updateRequest = pyqtSignal(dict)
    addBehaviourRequest = pyqtSignal(str)
    removeBehaviourRequest = pyqtSignal(str)
    addGoalRequest = pyqtSignal(str)
    removeGoalRequest = pyqtSignal(str)

    def __init__(self, context):
        super(Overview, self).__init__(context)
        
        self.__behaviour_widgets = {} # this stores all behaviours
        self.__goal_widgets = {}      # this stores all goals
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
        self.__sub = rospy.Subscriber(self.__plannerPrefix + "Planner/plannerStatus", PlannerStatus, self.plannerStatusCallback)
            
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
        self.removeBehaviourRequest.connect(self.removeBehaviourWidget)
        self.addGoalRequest.connect(self.addGoalWidget)
        self.removeGoalRequest.connect(self.removeGoalWidget)
        
    def updateGUI(self, newValues):
        self._widget.activationThresholdDoubleSpinBox.setValue(newValues["activationThreshold"])
        if not self._widget.activationThresholdDecayDoubleSpinBox.hasFocus():
            self._widget.activationThresholdDecayDoubleSpinBox.setValue(newValues["activationThresholdDecay"])
        self._widget.influencedSensorsLabel.setText(newValues["influencedSensors"])
        self._widget.runningBehavioursLabel.setText(newValues["runningBehaviours"])
        self.setPauseResumeButton(True) # if we receive updates the manager is running
    
    def addBehaviourWidget(self, name):
        self.__behaviour_widgets[name] = BehaviourWidget(name, self)
        self._widget.behaviourFrame.layout().addWidget(self.__behaviour_widgets[name])

    def removeBehaviourWidget(self, name):
        if name in self.__behaviour_widgets:
            behaviour = self.__behaviour_widgets[name]
            self._widget.behaviourFrame.layout().removeWidget(behaviour)
            behaviour.deleteLater()
            del self.__behaviour_widgets[name]
        
    def addGoalWidget(self, name):
        self.__goal_widgets[name] = GoalWidget(name, self)
        self._widget.goalFrame.layout().addWidget(self.__goal_widgets[name])

    def removeGoalWidget(self, name):
        if name in self.__goal_widgets:
            goal = self.__goal_widgets[name]
            self._widget.goalFrame.layout().removeWidget(goal)
            goal.deleteLater()
            del self.__goal_widgets[name]

    def setPauseResumeButton(self, running):
        if running:
            self._widget.pausePushButton.setText("pause")
            self._widget.pausePushButton.setChecked(False)
        else:
            self._widget.pausePushButton.setText("resume")
            self._widget.pausePushButton.setChecked(True)
    
    def pauseButtonCallback(self, status):
        try:
            if status == True:
                service_name = self.__plannerPrefix + '/' + 'Pause'
                rospy.logdebug("Waiting for service %s", service_name)
                rospy.wait_for_service(service_name)
                pauseRequest = rospy.ServiceProxy(service_name, Empty)
                pauseRequest()
                self.setPauseResumeButton(False)
            else:
                service_name = self.__plannerPrefix + '/' + 'Resume'
                rospy.logdebug("Waiting for service %s", service_name)
                rospy.wait_for_service(service_name)
                resumeRequest = rospy.ServiceProxy(service_name, Empty)
                resumeRequest()
                self.setPauseResumeButton(True)
        except Exception as e:
            rospy.logerr("error while toggling pause or resume: %s", str(e))
    
    def updateBehaviours(self, behaviours):
        """
        Add/remove behaviours to our management structure if necessary
        Update the behaviour Widget using its refresh() method.
        """
        updated_behaviour_names=[]
        for b in behaviours:
            if b.name not in self.__behaviour_widgets:
                self.addBehaviourRequest.emit(b.name)
                rospy.logdebug("Added behaviour %s",b.name)
            else:
                self.__behaviour_widgets[b.name].refresh(b)
            updated_behaviour_names.append(b.name)

        # check if we have to delete a behaviour widget
        for name, b in self.__behaviour_widgets.items():
            if name not in updated_behaviour_names:
                self.removeBehaviourRequest.emit(name)
                rospy.logdebug("Removed behaviour %s", name)

    def updateGoals(self, goals):
        """
        Add/remove goals to our management structure if necessary
        Update the goals Widget using its refresh() method.
        """
        updated_goal_names = []
        for g in goals:
            if g.name not in self.__goal_widgets:
                self.addGoalRequest.emit(g.name)
                rospy.logdebug("Added goal %s", g.name)
            else:
                self.__goal_widgets[g.name].refresh(g)
            updated_goal_names.append(g.name)

        # check if we have to delete a goal widget
        for name, g in self.__goal_widgets.items():
            if name not in updated_goal_names:
                self.removeGoalRequest.emit(name)
                rospy.logdebug("Removed goal %s", name)

    def setActivationThresholdDecay(self):
        rospy.loginfo("setting activationThresholdDecay to %f", self._widget.activationThresholdDecayDoubleSpinBox.value())
        rospy.set_param("activationThresholdDecay", self._widget.activationThresholdDecayDoubleSpinBox.value())
    
    def setPlannerPrefix(self):
        self.__sub.unregister()
        self.__plannerPrefix = self._widget.plannerPrefixEdit.text()
        status_topic_name =  self.__plannerPrefix + '/' + "Planner/plannerStatus"
        rospy.loginfo("subscribing to %s", status_topic_name)
        self.__sub = rospy.Subscriber(status_topic_name, PlannerStatus, self.plannerStatusCallback)
    
    def plannerStatusCallback(self, msg):
        try:
            self.updateBehaviours(msg.behaviours)
            self.updateGoals(msg.goals)
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
