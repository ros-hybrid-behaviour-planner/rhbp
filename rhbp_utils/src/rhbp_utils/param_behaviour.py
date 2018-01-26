"""
Created on 26.01.2018

@author: hrabia
"""

import rospy

from behaviour_components.behaviours import BehaviourBase


import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME)


class ParamSetterBehaviour(BehaviourBase):
    """
    Behaviour that sets a ROS param after it is activated
    """

    def __init__(self, name, param, new_value, **kwargs):
        """
        :param name: behaviour name
        :param param: param name that is changed
        :param new_value: value that will be set after activation
        :param kwargs: further general BehaviourBase arguments
        """

        super(ParamSetterBehaviour, self) \
            .__init__(name=name, **kwargs)

        self._param = param
        self._new_value = new_value

    def start(self):
        rhbplog.logdebug("Setting param '%s' to '%s'", self._param, self._new_value)

        rospy.set_param(self._param, self._new_value)


