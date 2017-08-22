'''
Created on 22.08.2017

@author: hrabia
'''
from __future__ import division # force floating point division when using plain /
import rospy
from .pddl import PDDL
from rhbp_core.msg import Wish as WishMsg

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.planning')


class Effect(object):
    """
    This class models effects and their combinations.
    All effects (correlations) are assumed to happen simultaneously except otherwise stated.
    """

    def __init__(self, sensorName, indicator, sensorType=bool, condition=None):
        """
        :param sensorName: name of the influenced sensor/activator pair
        :type sensorName: str
        :param indicator: float value indicating the strength and the direction of the influence, commonly between [-1,1], 0 refers to no influence
        :type indicator: float
        :param sensorType: bool, int or float
        :param condition: manually added conditional effect
        :type condition: str
        """
        self.sensorName = sensorName
        self.indicator = indicator
        self.sensorType = sensorType
        self.condition = condition

    def getEffectPDDL(self):
        pddl = PDDL(statement="(")
        obr = 1  # count opened brackets
        if self.condition is not None:
            pddl.statement += "when ({0}) (".format(self.condition)
            obr += 1
        if self.sensorType is bool:
            pddl.statement += self.sensorName if self.indicator > 0 else "not ({0})".format(self.sensorName)
            pddl.predicates.add(self.sensorName)  # TODO: What about other predicates employed in conditions by user??
        else:  # its numeric and not bool
            pddl.statement += "{0} ({1}) {2}".format("increase" if self.indicator > 0.0 else "decrease",
                                                     self.sensorName, abs(self.indicator))
            pddl.functions.add(self.sensorName)  # TODO: What about other functions employed in conditions by user??
        pddl.statement += ")" * obr  # close the brackets
        return pddl

class Wish(object):

    def __init__(self, sensor_name, indicator, activator_name=None):
        self.sensor_name = sensor_name
        self.indicator = indicator
        self.activator_name = activator_name

    def get_wish_msg(self):
        msg = WishMsg()
        msg.indicator=self.indicator
        msg.sensorName=self.sensor_name
        msg.activatorName = self.activator_name
        return msg

    @staticmethod
    def from_wish_msg(wish_msg):
        """
        Create this Wish object from Wish msg type
        :param wish_msg: 
        :type wish_msg: WishMsg
        :return: 
        """
        return Wish(sensor_name=wish_msg.sensorName, indicator=wish_msg.indicator, activator_name=wish_msg.activatorName)

    def get_pddl_effect_name(self):
        return self.activator_name + '_' + self.sensor_name

    def __str__(self):
        return "Wish(s:{},i:{},a:{})".format(str(self.sensor_name), str(self.indicator), str(self.activator_name))