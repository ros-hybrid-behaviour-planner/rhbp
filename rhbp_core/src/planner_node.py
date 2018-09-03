#! /usr/bin/env python2
"""
Created on 13.04.2015

@author: wypler, hrabia
"""
import sys
import rospy
from behaviour_components.managers import Manager
from rhbp_core.srv import SetStepping, SetSteppingResponse, GetStepping, GetSteppingResponse
from std_srvs.srv import Empty, EmptyResponse
from rospy.exceptions import ROSInterruptException


class ManagerNode(object):
    """
    ROS node wrapper for the rhbp manager/planner
    """

    def __init__(self, manager_prefix=""):

        rospy.init_node('planner_node', log_level=rospy.WARN)

        prefix = rospy.get_param("~prefix", manager_prefix)
        self._manager = Manager(prefix=prefix)
        self.rate = rospy.Rate(rospy.get_param("~frequency", 1))

        self.automatic_stepping = rospy.get_param("~automatic_stepping", True)

        if not self.automatic_stepping:
            rospy.logwarn("Started in manual stepping mode")

        self._init_services(prefix)

        rospy.on_shutdown(self._unregister)  # cleanup hook

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

        rospy.loginfo("Automatic Stepping changed to " + str(self.automatic_stepping))

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
            self._manager.step(force=True)
        else:
            rospy.logwarn("No manual stepping if automatic stepping is enabled and manager is not paused")
        return EmptyResponse()

    def run(self):
        """
        Executing the node after initialization
        """
        while not rospy.is_shutdown():
            if self.automatic_stepping:
                self._manager.step()
            else:
                self._manager.send_discovery()
            self.rate.sleep()


if __name__ == '__main__':

    prefix = ""

    for arg in sys.argv:
        if arg.startswith('prefix:='):
            prefix = arg[len('prefix:='):]
            break

    node = ManagerNode(manager_prefix=prefix)

    try:
        node.run()
    except ROSInterruptException:
        rospy.loginfo("Planner node shut down")
