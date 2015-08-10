## rqt widget showing global behaviour planner attributes and providing a container for behaviour widgets
#Created on 10.08.2015
#@author: stephan

import os
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from behaviourWidget import BehaviourWidget
from goalWidget import GoalWidget
from behaviourPlannerPython.msg import PlannerStatus



class Overview(Plugin):

    def __init__(self, context):
        super(Overview, self).__init__(context)
        
        self.__behaviours = {} # this stores all behaviours
        self.__goals = {}      # this stores all goals
        self.__plannerPrefix = ""
        
        # Give QObjects reasonable names
        self.setObjectName('BehaviourPlannerOverview')

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
        self.__sub = rospy.Subscriber("/" + self.__plannerPrefix + "Planer/plannerStatus", PlannerStatus, self.plannerStatusCallback)
            
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this node
        ui_file = os.path.join(rospkg.RosPack().get_path('behaviourPlannerPython'), 'src', 'rqt_planner_gui', 'resource', 'overview.ui')
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
        ### demo stuff ###
        self.addBehaviour("startBehaviour")
        self.addBehaviour("landBehaviour")
        rospy.loginfo("started")
        self.addGoal("returnedHomeGoal")

    
    def addBehaviour(self, name):
        if not name in self.__behaviours:
            self.__behaviours[name] = BehaviourWidget(name)
            self._widget.behaviourGroupBox.layout().addWidget(self.__behaviours[name])
        else:
            print name, "was already registered"
    
    def addGoal(self, name):
        if not name in self.__goals:
            self.__goals[name] = GoalWidget(name)
            self._widget.goalGroupBox.layout().addWidget(self.__goals[name])
        else:
            print name, "was already registered"
    
    def setActivationThresholdDecay(self):
        rospy.loginfo("setting activationThresholdDecay to %f", self._widget.activationThresholdDecayDoubleSpinBox.value())
        rospy.set_param("activationThresholdDecay", self._widget.activationThresholdDecayDoubleSpinBox.value())
    
    def setPlannerPrefix(self):
        self.__sub.unregister()
        self.__plannerPrefix = self._widget.plannerPrefixEdit.text()
        rospy.loginfo("subscribing to %s", "/" + self.__plannerPrefix + "Planer/plannerStatus")
        self.__sub = rospy.Subscriber("/" + self.__plannerPrefix + "Planer/plannerStatus", PlannerStatus, self.plannerStatusCallback)

    def onBehaviourUpdate(self):
        rospy.loginfo("onBehaviourUpdate")
    
    def plannerStatusCallback(self, msg):
        rospy.logdebug("received %s", msg)
        self._widget.activationThresholdDoubleSpinBox.setValue(msg.activationThreshold)
        if not self._widget.activationThresholdDecayDoubleSpinBox.hasFocus():
            self._widget.activationThresholdDecayDoubleSpinBox.setValue(msg.activationThresholdDecay)
        # TODO: do all the behaviour and goal stuff

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
