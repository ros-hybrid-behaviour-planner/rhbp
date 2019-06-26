#! /usr/bin/env python2
"""
Created on 25.06.2019

@author: hrabia

"""
import sys
from behaviour_components.manager_node import ManagerNode
from decomposition_components.managers import Manager
from rospy.exceptions import ROSInterruptException

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.delegation')


class DelegationManagerNode(ManagerNode):
    """
    Custom ManagerNode creating the specific DelegationManager instead of the default one.
    """

    def _create_manager(self, prefix):
        """
        overwritten factory method for creating a specific manager
        :param prefix: manager prefix
        :return: manager instance
        """
        return Manager(prefix=prefix)


if __name__ == '__main__':

    prefix = ""

    for arg in sys.argv:
        if arg.startswith('prefix:='):
            prefix = arg[len('prefix:='):]
            break

    node = DelegationManagerNode(manager_prefix=prefix)

    try:
        node.run()
    except ROSInterruptException:
        rhbplog.loginfo("Planner node shut down")
