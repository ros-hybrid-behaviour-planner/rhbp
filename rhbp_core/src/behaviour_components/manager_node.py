#! /usr/bin/env python2
"""
Created on 13.04.2015

@author: wypler, hrabia
"""
import rospy
from behaviour_components.managers import Manager
from rhbp_core.srv import SetStepping, SetSteppingResponse, GetStepping, GetSteppingResponse
from std_srvs.srv import Empty, EmptyResponse

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME)


class ManagerNode(object):
    """
    ROS node runtime wrapper for the rhbp manager/planner
    """

    def __init__(self, manager_prefix=""):

        rospy.init_node('planner_node', log_level=rospy.WARN)

        prefix = rospy.get_param("~prefix", manager_prefix)
        self._manager = self._create_manager(prefix=prefix)
        frequency = rospy.get_param("~frequency", 1)
        self.rate = rospy.Rate(frequency)

        self.automatic_stepping = rospy.get_param("~automatic_stepping", True)
        self.guarantee_decision = rospy.get_param("~guarantee_decision", False)

        rhbplog.loginfo("Planner node configuration: prefix: %s; frequency:%d; automatic_stepping:%s; "
                        "guarantee_decision:%s", prefix, frequency, self.automatic_stepping, self.guarantee_decision)

        if not self.automatic_stepping:
            rhbplog.logwarn("Started in manual stepping mode")

        self._init_services(prefix)

        rospy.on_shutdown(self._unregister)  # cleanup hook

    def _create_manager(self, prefix):
        """
        factory method for creating the actual manager instance, this simplifies custom implementations of the planner
        node
        :param prefix: manager prefix
        :return: Manager
        """
        return Manager(prefix=prefix)

    def _unregister(self):
        """
        Unregister all services etc. Instance is not usable anymore afterwards
        """
        self._manager.pause()
        self._set_automatic_stepping_service.shutdown()
        self._get_automatic_stepping_service.shutdown()
        self._stepping_service.shutdown()
        self._manager.unregister()

    def _init_services(self, prefix):
        """
        init all services handlers
        :param prefix: manager prefix
        """
        self._set_automatic_stepping_service = rospy.Service(prefix + '/set_automatic_stepping', SetStepping,
                                                             self._set_stepping_callback)
        self._get_automatic_stepping_service = rospy.Service(prefix + '/get_automatic_stepping', GetStepping,
                                                             self._get_stepping_callback)
        self._stepping_service = rospy.Service(prefix + '/step', Empty, self._step_callback)

    def _set_stepping_callback(self, request):
        """
        Callback service for enabling or disabling automatic stepping in the given frequency
        :param request:
        """
        self.automatic_stepping = request.automatic_stepping

        rhbplog.loginfo("Automatic Stepping changed to " + str(self.automatic_stepping))

        return SetSteppingResponse()

    def _get_stepping_callback(self, request):
        """
        Callback service for getting the current automatic stepping setting
        :param request:
        """
        response = GetSteppingResponse(self.automatic_stepping)

        return response

    def _step_callback(self, request):
        """
        Service callback for manual planning/manager steps
        :param request:
        """
        if not self.automatic_stepping or self._manager.paused:
            self._manager.step(force=True, guarantee_decision=self.guarantee_decision)
        else:
            rhbplog.logwarn("No manual stepping if automatic stepping is enabled and manager is not paused")
        return EmptyResponse()

    def run(self):
        """
        Executing the node after initialization
        """
        while not rospy.is_shutdown():
            if self.automatic_stepping:
                self._manager.step(guarantee_decision=self.guarantee_decision)
            else:
                self._manager.send_discovery()
            self.rate.sleep()
