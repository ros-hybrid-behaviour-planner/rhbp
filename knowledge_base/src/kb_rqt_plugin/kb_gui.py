## rqt widget to monitor the knowledgebase
#Created on 11.01.2017
#@author: hrabia

import os
import time
import threading
import copy
import rospy
import rospkg
import traceback
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QStyledItemDelegate
from PyQt5.QtCore import pyqtSignal, QAbstractTableModel, QModelIndex, Qt, QVariant
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.update_handler import KnowledgeBaseFactCache
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from knowledge_base.msg import DiscoverInfo


class ItemDelegate(QStyledItemDelegate):
    """
    Helper Class that allows to get a signal on a start and finish of cell editing
    """
    cellEditingStarted = pyqtSignal(int, int)
    cellEditingFinished = pyqtSignal(int, int)

    def createEditor(self, parent, option, index):
        """
        This is triggered when editing starts
        """
        result = super(ItemDelegate, self).createEditor(parent, option, index)
        if result:
            self.cellEditingStarted.emit(index.row(), index.column())
        return result

    def setModelData (self, editor, model, index):
        """
        This is triggered when editing is finished
        """
        super(ItemDelegate, self).setModelData(editor, model, index)
        self.cellEditingFinished.emit(index.row(), index.column())


class FactTableModel(QAbstractTableModel):
    """
    Simple QT table model for Knowledge Base facts
    """

    def __init__(self, kb_name, parent=None, *args):
        super(FactTableModel, self).__init__(parent)
        self.facts = None
        self._kb_name = kb_name
        self._kb_client = KnowledgeBaseClient(knowledge_base_name=self._kb_name, timeout=0.5)

    def update(self, new_facts):
        """
        Use the method to update the fact/data base
        :param new_facts: new list of fact tuples
        """
        if new_facts:
            new_facts.sort()
        self.layoutAboutToBeChanged.emit()
        self.facts = new_facts
        self.layoutChanged.emit()

    def clear(self):
        """
        Clear current fact base
        """
        self.update(None)

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
        if role == Qt.DisplayRole or role == Qt.EditRole:
            i = index.row()
            j = index.column()
            if j < len(self.facts[i]):
                return '{0}'.format(self.facts[i][j])
            else:
                return QVariant()
        else:
            return QVariant()

    def setData(self, index, value, role):
        """
        Update remote knowledge base data
        """
        try:

            pattern = copy.deepcopy(self.facts[index.row()])

            current_fact = list(pattern)

            if index.column() >= len(current_fact):
                for i in range(0, index.column()-len(current_fact) + 1):
                    current_fact.append("")
            current_fact[index.column()] = str(value)

            rospy.logdebug("Changing knowledge: \n" + str(pattern) + "\n --to-- \n" + str(current_fact))

            return self._kb_client.update(pattern=pattern, new=current_fact)
        except Exception:
            rospy.logerr(traceback.format_exc())
            return False

    def flags(self, index):
        return Qt.ItemIsEditable | Qt.ItemIsEnabled | Qt.ItemIsSelectable

    @property
    def kb_name(self):
        return self._kb_name

    @kb_name.setter
    def kb_name(self, value):
        if self._kb_name != value:
            self._kb_name = value
            self._kb_client = KnowledgeBaseClient(knowledge_base_name=value, timeout=0.5)


class KbUi(Plugin):
    """
    KnowledgeBase Monitor Plugin main class

    The Plugin allows to connect to a running KB and enables displaying and editing of the current fact base
    """

    _update_kb_facts = pyqtSignal()
    _set_kb = pyqtSignal(str,str)
    _update_discover = pyqtSignal(DiscoverInfo)

    def __init__(self, context):
        super(KbUi, self).__init__(context)

        self.setObjectName('KnowledgeBase UI')

        self.__edit_lock = threading.Lock()
        self.__connect_lock = threading.Lock()

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
        self._widget.setObjectName('KnowledgeBase Monitor')
        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget).
        # This is useful when you open multiple plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.kbConnectPushButton.clicked.connect(self._connect_button_callback)

        self._fact_table_model = FactTableModel(kb_name=self._kb_name, parent=self._widget)

        self._widget.kbTableView.setModel(self._fact_table_model)

        self._widget.kbPatternlineEdit.setToolTip("Separate tuples with , and use * as placeholder, keep empty to "
                                                  "match all tuples")

        # configuring TableView event for item edit
        self._cell_edit_delegate = ItemDelegate(self._widget)
        self._cell_edit_delegate.cellEditingStarted.connect(self._callback_kb_editing_started)
        self._cell_edit_delegate.cellEditingFinished.connect(self._callback_kb_editing_finished)
        self._widget.kbTableView.setItemDelegate(self._cell_edit_delegate)

        # connecting signals
        self._update_kb_facts.connect(self._callback_update_fact_update)
        # this is necessary to correctly (un)register in KnowledgeBaseFactCache, directly using .emit returns
        #  uncomparable objects
        self._update_kb_facts_emit_func = self._update_kb_facts.emit
        self._update_discover.connect(self._update_discovery)
        self._set_kb.connect(self._set_knowledge_base)

        # subscribe to our information source
        self.__kb_discovery = rospy.Subscriber(KnowledgeBase.DISCOVERY_TOPIC_NAME, DiscoverInfo,
                                               self._update_discover.emit)

    def _set_knowledge_base(self, kb_name, pattern_text=""):
        """
        set the knowledge base we want to connect to
        :param kb_name: name of the KB
        :param pattern_text: pattern for the KB update as text, komma separates tuples
        """
        with self.__connect_lock:
            self._widget.kbStatusLabel.setText("Trying to connect to {0}".format(kb_name))
            if hasattr(self, "_kb_cache") and self._kb_cache:                
                self._kb_cache.remove_update_listener(self._update_kb_facts_emit_func)
                self._fact_table_model.clear()
            self._kb_name = kb_name
            if len(pattern_text) > 0:
                pattern = pattern_text.split(",")
            else:
                pattern = ()
            self._kb_cache = KnowledgeBaseFactCache(pattern=pattern, knowledge_base_name=self._kb_name, timeout=2)
            self._update_discovery(DiscoverInfo(kb_name=kb_name))
            self._kb_cache.add_update_listener(self._update_kb_facts_emit_func)

            # manually update once
            try:
                self._callback_update_fact_update()  # not using emit here to get possible exceptions
            except rospy.ROSException as e:
                rospy.logwarn(e)
                self._widget.kbStatusLabel.setText("Could not connect to '{0}'".format(kb_name))

    def _connect_button_callback(self):
        self.start_kb_update_thread(self._widget.kbNodeComboBox.currentText(), self._widget.kbPatternlineEdit.text())

    def _callback_kb_editing_started(self, row, column):
        self.__edit_lock.acquire()

    def _callback_kb_editing_finished(self, row, column):
        self.__edit_lock.release()

    def _callback_update_fact_update(self):
        if self.__edit_lock.acquire(False):
            try:                
                facts = self._kb_cache.get_all_matching_facts()
                self._fact_table_model.update(new_facts=facts)
                self._fact_table_model.kb_name = self._kb_name
                self._widget.kbStatusLabel.setText("Last Update: {0}".format(time.ctime()))
            finally:
                self.__edit_lock.release()

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
            kb_name = stored_kb_name
        else:
            kb_name = KnowledgeBase.DEFAULT_NAME

        self.start_kb_update_thread(kb_name)

    def start_kb_update_thread(self, kb_name, pattern_text=""):
        """
        starting a background thread for kb update (new kb or pattern)
        :param kb_name: name of the knowledge base
        :param pattern_text: pattern to use for updates
        """

        # update with local function and thread
        def run_kb_update():
            try:
                self._set_knowledge_base(kb_name=kb_name, pattern_text=pattern_text)
            except rospy.ROSException as e:
                rospy.logwarn(e)
                self._widget.kbStatusLabel.setText("Error: ".format(str(e)))

        initial_update_thread = threading.Thread(target=run_kb_update)
        initial_update_thread.start()
