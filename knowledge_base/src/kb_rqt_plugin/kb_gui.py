## rqt widget to monitor the knowledgebase
#Created on 11.01.2017
#@author: hrabia

import os
import time
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal, QAbstractTableModel, QModelIndex, Qt, QVariant
from threading import Lock
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.update_handler import KnowledgeBaseFactCache
from knowledge_base.msg import DiscoverInfo


class FactTableModel(QAbstractTableModel):
    """
    Simple QT table model for Knowledge Base facts
    """

    def __init__(self, parent=None, *args):
        super(FactTableModel, self).__init__()
        self.facts = None

    def update(self, new_facts):
        """
        Use the method to update the fact/data base
        :param new_facts: new list of fact tuples
        """
        self.layoutAboutToBeChanged.emit()
        self.facts = new_facts
        self.layoutChanged.emit()

    def rowCount(self, parent=QModelIndex()):
        if self.facts:
            return len(self.facts)
        else:
            return 0

    def columnCount(self, parent=QModelIndex()):

        if self.facts:
            max_column = 0
            for fact in self.facts:
                max_column = max(max_column, len(fact))
            return max_column
        else:
            return 0

    def data(self, index, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            i = index.row()
            j = index.column()
            if j < len(self.facts[i]):
                return '{0}'.format(self.facts[i][j])
            else:
                return QVariant()
        else:
            return QVariant()

    def flags(self, index):
        return Qt.ItemIsEnabled


class KbUi(Plugin):
    """
    KnowledgeBase Monitor Plugin main class
    """

    _update_kb = pyqtSignal()
    _update_discover = pyqtSignal(DiscoverInfo)

    def __init__(self, context):
        super(KbUi, self).__init__(context)
        self._kb_name = KnowledgeBase.DEFAULT_NAME

        # Give QObjects reasonable names
        self.setObjectName('kb_monitor')

        # Storing discovered knowledge base names
        self._kb_node_collection = []

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this node
        ui_file = os.path.join(rospkg.RosPack().get_path('knowledge_base'), 'src', 'kb_rqt_plugin', 'resource', 'main.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('KnowledgeBase GUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.kbConnectPushButton.clicked.connect(self._connect_button_callback)

        self._fact_table_model = FactTableModel(self._widget)

        self._widget.kbTableView.setModel(self._fact_table_model)

        self._update_kb.connect(self._callback_update_kb)

        self._update_discover.connect(self._update_discovery)

        # subscribe to our information source
        self.__kb_discovery = rospy.Subscriber(KnowledgeBase.DISCOVERY_TOPIC_NAME, DiscoverInfo,
                                               self._update_discover.emit)

        self._set_knowledge_base(kb_name=self._kb_name)

    def _set_knowledge_base(self, kb_name):
        """
        set the knowledge base we want to connect to
        :param kb_name: name of the KB
        """
        if hasattr(self, "_kb_cache") and self._kb_cache:
            self._kb_cache.remove_update_listener(self._update_kb.emit)
        self._kb_name = kb_name
        self._kb_cache = KnowledgeBaseFactCache(pattern=(), knowledge_base_name=self._kb_name)
        self._update_discovery(DiscoverInfo(kb_name=kb_name))
        self._kb_cache.add_update_listener(self._update_kb.emit)
        # manually update once
        self._callback_update_kb()

    def _connect_button_callback(self):
        self._set_knowledge_base(self._widget.kbNodeComboBox.currentText())

    def _callback_update_kb(self):
        facts = self._kb_cache.get_all_matching_facts()
        self._fact_table_model.update(new_facts=facts)
        self._widget.kbStatusLabel.setText("Last Update: {0}".format(time.ctime()))

    def _update_discovery(self, kb_discovery):
        """
        Update kb list
        :param kb_discovery: 
        :type kb_discovery DiscoverInfo
        """
        kb_name = kb_discovery.kb_name
        if kb_name not in self._kb_node_collection:
            self._kb_node_collection.append(kb_name)
            self._widget.kbNodeComboBox.addItem(kb_name)
            rospy.loginfo("Added kb node: %s", kb_name)
            self._widget.kbStatusLabel.setText("Added new KB node '{0}'".format(kb_name))

    def shutdown_plugin(self):
        self.__kb_discovery.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        rospy.loginfo("saving knowledge_base setting")
        instance_settings.set_value("knowledge_base_name", self._kb_name)

    def restore_settings(self, plugin_settings, instance_settings):
        rospy.loginfo("restoring knowledge_base setting")
        stored_kb_name = instance_settings.value("knowledge_base_name")
        if type(stored_kb_name) == unicode:
            stored_kb_name = stored_kb_name.encode('ascii', 'ignore')
        if stored_kb_name:
            rospy.loginfo("Using stored prefix: %s", stored_kb_name)
            self._set_knowledge_base(stored_kb_name)