"""
Created on 26.01.2018

@author: hrabia
"""

from behaviour_components.sensors import Sensor
import rospy

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME)


class ParamSensor(Sensor):
    """
    Sensor for retrieving param information from all ROS namespaces (private, global)
    """

    def __init__(self, name, param,  optional=False, initial_value=None):
        """
        :param name: name of the sensor
        :param param: name of the parameter
        """

        super(ParamSensor, self).__init__(name=name, optional=optional, initial_value=initial_value)

        self._param = param

    def sync(self):
        try:
            value = rospy.get_param(self._param)
        except KeyError:
            rhbplog.logwarn("Parameter '%s' not available", self._param)
            value = self._initial_value

        super(ParamSensor, self).update(newValue=value)

        return super(ParamSensor, self).sync()

    @property
    def param(self):
        return self._param
