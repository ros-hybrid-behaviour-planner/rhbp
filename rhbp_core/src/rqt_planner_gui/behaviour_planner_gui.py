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
from rhbp_core.msg import PlannerStatus, DiscoverInfo
from rhbp_core.srv import SetStepping, GetStepping
from PyQt5.QtCore import pyqtSignal
from behaviour_components.managers import Manager
from threading import Lock


class Overview(Plugin):
    updateRequest = pyqtSignal(dict)
    addBehaviourRequest = pyqtSignal(str)
    removeBehaviourRequest = pyqtSignal(str)
    addGoalRequest = pyqtSignal(str)
    removeGoalRequest = pyqtSignal(str)
    updateDiscoveryRequest = pyqtSignal(str)

    def __init__(self, context):
        super(Overview, self).__init__(context)
        
        self.__behaviour_widgets = {} # this stores all behaviours
        self.__behaviour_names = []
        self.__goal_widgets = {}      # this stores all goals
        self.__goal_names = []
        self.__plannerPrefix = ""
        
        # Give QObjects reasonable names
        self.setObjectName('behaviour_planner_overview')

        self._planner_prefix_collection = []

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-p", "--plannerPrefix", dest="plannerPrefix", help="Specify the planner prefix")
        args, unknowns = parser.parse_known_args(context.argv())
        if args.plannerPrefix:
            self.__plannerPrefix = args.plannerPrefix
            self._planner_prefix_collection.append(self.__plannerPrefix)
            rospy.loginfo("using planner prefix %s", self.__plannerPrefix)

        self.__behaviour_lock = Lock()
        self.__goal_lock = Lock()

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
        self._widget.plannerPrefixPushButton.clicked.connect(self.set_planner_prefix_callback)
        self._widget.pausePushButton.toggled.connect(self.pauseButtonCallback)
        self._widget.stepPushButton.clicked.connect(self.step_push_button_callback)
        self._widget.automaticSteppingCheckBox.toggled.connect(self._automatic_stepping_checkbox_Callback)
        #this fire fore every key input
        #self._widget.plannerPrefixComboBox.editTextChanged.connect(self.set_planner_prefix_callback)
        self._widget.plannerPrefixComboBox.activated.connect(self.set_planner_prefix_callback)

        # Connect signal so we can refresh Widgets from the main thread        
        self.updateRequest.connect(self.updateGUI)
        self.addBehaviourRequest.connect(self.addBehaviourWidget)
        self.removeBehaviourRequest.connect(self.removeBehaviourWidget)
        self.addGoalRequest.connect(self.addGoalWidget)
        self.removeGoalRequest.connect(self.removeGoalWidget)
        self.updateDiscoveryRequest.connect(self._update_discovery)

        # subscribe to our information source
        self.__sub_planner_status = None
        self.set_planner_prefix(self.__plannerPrefix)

        self.__sub_planner_discovery = rospy.Subscriber(Manager.MANAGER_DISCOVERY_TOPIC, DiscoverInfo, self._planner_discovery_callback)
        
    def updateGUI(self, newValues):
        self._widget.activationThresholdDoubleSpinBox.setValue(newValues["activationThreshold"])
        if not self._widget.activationThresholdDecayDoubleSpinBox.hasFocus():
            self._widget.activationThresholdDecayDoubleSpinBox.setValue(newValues["activationThresholdDecay"])
        self._widget.influencedSensorsLabel.setText(newValues["influencedSensors"])
        self._widget.runningBehavioursLabel.setText(newValues["runningBehaviours"])
        self._widget.currentStepLabel.setText(str(newValues["stepCounter"]))
        self.setPauseResumeButton(True)  # if we receive updates the manager is running
    
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
        with self.__behaviour_lock:
            updated_behaviour_names = []
            for b in behaviours:
                if b.name not in self.__behaviour_names:
                    self.__behaviour_names.append(b.name)
                    self.addBehaviourRequest.emit(b.name)
                    rospy.logdebug("Added behaviour %s", b.name)
                else:
                    self.__behaviour_widgets[b.name].refresh(b)
                updated_behaviour_names.append(b.name)

            # check if we have to delete a behaviour widget
            for name in self.__behaviour_names:
                if name not in updated_behaviour_names:
                    self.__behaviour_names.remove(name)
                    self.removeBehaviourRequest.emit(name)
                    rospy.logdebug("Removed behaviour %s", name)

    def updateGoals(self, goals):
        """
        Add/remove goals to our management structure if necessary
        Update the goals Widget using its refresh() method.
        """
        with self.__goal_lock:
            updated_goal_names = []
            for g in goals:
                if g.name not in self.__goal_names:
                    self.__goal_names.append(g.name)
                    self.addGoalRequest.emit(g.name)
                    rospy.logdebug("Added goal %s", g.name)
                else:
                    self.__goal_widgets[g.name].refresh(g)
                updated_goal_names.append(g.name)

            # check if we have to delete a goal widget
            for name in self.__goal_names:
                if name not in updated_goal_names:
                    self.__goal_names.remove(name)
                    self.removeGoalRequest.emit(name)
                    rospy.logdebug("Removed goal %s", name)

    def step_push_button_callback(self):
        service_name = self.__plannerPrefix + '/' + 'step'
        rospy.logdebug("Waiting for service %s", service_name)
        rospy.wait_for_service(service_name, timeout=1)
        stepRequest = rospy.ServiceProxy(service_name, Empty)
        stepRequest()

    def _automatic_stepping_checkbox_Callback(self, status):
        self._widget.stepPushButton.setEnabled(status)

        service_name = self.__plannerPrefix + '/' + 'set_automatic_stepping'
        rospy.logdebug("Waiting for service %s", service_name)
        rospy.wait_for_service(service_name, timeout=1)
        set_stepping = rospy.ServiceProxy(service_name, SetStepping)
        set_stepping(status)

    def _is_automatic_stepping_enabled(self):
        service_name = self.__plannerPrefix + '/' + 'get_automatic_stepping'
        rospy.logdebug("Waiting for service %s", service_name)
        rospy.wait_for_service(service_name, timeout=1)
        get_stepping = rospy.ServiceProxy(service_name, GetStepping)
        ret = get_stepping()
        if ret and ret.automatic_stepping:
            return True
        else:
            return False

    def setActivationThresholdDecay(self):
        rospy.loginfo("setting activationThresholdDecay to %f", self._widget.activationThresholdDecayDoubleSpinBox.value())
        rospy.set_param("activationThresholdDecay", self._widget.activationThresholdDecayDoubleSpinBox.value())
    
    def set_planner_prefix_callback(self):
        self.set_planner_prefix(self._widget.plannerPrefixComboBox.currentText())

    def set_planner_prefix(self, planner_prefix):
        self.__plannerPrefix = planner_prefix
        if self.__sub_planner_status:
            self.__sub_planner_status.unregister()
        status_topic_name = planner_prefix + '/' + "Planner/plannerStatus"
        rospy.loginfo("subscribing to %s", status_topic_name)
        self.__sub_planner_status = rospy.Subscriber(status_topic_name, PlannerStatus, self.plannerStatusCallback)
        # reset listed behaviours/goals
        self.updateBehaviours([])
        self.updateGoals([])
        try:
            enabled = self._is_automatic_stepping_enabled()
            rospy.loginfo("Automatic stepping enabled: " + str(enabled))
            self._widget.automaticSteppingCheckBox.setEnabled(True)
            self._widget.stepPushButton.setEnabled(not enabled)
        except:
            self._widget.automaticSteppingCheckBox.setEnabled(False)
            self._widget.stepPushButton.setEnabled(False)

    def _update_discovery(self, manager_prefix):
        """
        Update prefix list
        :param manager_prefix: 
        :type manager_prefix DiscoverInfo
        """
        if manager_prefix not in self._planner_prefix_collection:
            self._planner_prefix_collection.append(manager_prefix)
            self._widget.plannerPrefixComboBox.addItem(manager_prefix)
            rospy.loginfo("Added prefix: %s", manager_prefix)

    def _planner_discovery_callback(self, msg):
        """
        discovery message callback handler
        :param msg: 
        :type msg: DiscoverInfo
        """
        self.updateDiscoveryRequest.emit(msg.manager_prefix)

    def plannerStatusCallback(self, msg):
        try:
            self.updateBehaviours(msg.behaviours)
            self.updateGoals(msg.goals)
            self.updateRequest.emit({
                                     "activationThreshold" : msg.activationThreshold,
                                     "activationThresholdDecay" : msg.activationThresholdDecay,
                                     "influencedSensors" : ", ".join(msg.influencedSensors),
                                     "runningBehaviours" : ", ".join(msg.runningBehaviours),
                                     "stepCounter": msg.stepCounter
                                    }) 
        except Exception as e:
            rospy.logerr("%s", e)
    
    @property
    def plannerPrefix(self):
        return self.__plannerPrefix
        
    def shutdown_plugin(self):
        self.__sub_planner_status.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        rospy.loginfo("saving plannerPrefix setting")
        instance_settings.set_value("plannerPrefix", self.__plannerPrefix)

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        rospy.loginfo("restoring plannerPrefix setting")
        stored_planner_prefix = instance_settings.value("plannerPrefix")
        if type(stored_planner_prefix) == unicode:
            stored_planner_prefix = stored_planner_prefix.encode('ascii','ignore')
        if stored_planner_prefix:
            rospy.loginfo("Using stored prefix: %s", stored_planner_prefix)
            self._update_discovery(stored_planner_prefix)
            self.set_planner_prefix_callback()