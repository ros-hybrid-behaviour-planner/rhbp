'''
Created on 22.08.2017

@author: hrabia
'''
from __future__ import division # force floating point division when using plain /
from .pddl import PDDL, get_pddl_effect_name, create_valid_pddl_name
from rhbp_core.msg import Wish as WishMsg
from rhbp_core.msg import Correlation as CorrelationMsg

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.planning')


class Effect(object):
    """
    This class models effects and their combinations.
    All effects (correlations) are assumed to happen simultaneously except otherwise stated.
    """

    def __init__(self, sensor_name, indicator, sensor_type=bool, activator_name='', condition=''):
        """
        :param sensor_name: name of the influenced sensor
        :type sensor_name: str
        :param indicator: float value indicating the strength and the direction of the influence, commonly between [-1,1], 0 refers to no influence
        :type indicator: float
        :param activator_name: name of the influenced activator if a particular one is specified
        :type activator_name: str
        :param sensor_type: bool, int or float
        :param condition: manually added conditional effect
        :type condition: str
        """
        self.sensor_name = create_valid_pddl_name(sensor_name)
        self.activator_name = activator_name
        self.sensor_type = str(sensor_type)
        self.condition = condition
        if isinstance(indicator, bool):
            if indicator:
                self.indicator = 1.0
            else:
                self.indicator = -1.0
        else:
            self.indicator = indicator

    @staticmethod
    def from_msg(msg):
        """
        Create this Effect object from Correlation msg type
        :param msg: 
        :type msg: CorrelationMsg
        :return: Effect()
        """
        return Effect(sensor_name=msg.sensorName, indicator=msg.indicator, sensor_type=msg.sensorType,
                      activator_name=msg.activatorName, condition=msg.condition)

    def get_msg(self):
        """
        Get msg representation of this class
        :return: CorrelationMsg()
        """
        msg = CorrelationMsg()
        msg.sensorName = self.sensor_name
        msg.indicator = self.indicator
        msg.sensorType = self.sensor_type
        msg.activatorName = self.activator_name
        msg.condition = self.condition
        return msg

    def getEffectPDDL(self):
        """
        Generate the pddl effect string
        :return: str
        """

        pddl = PDDL(statement="(")
        obr = 1  # count opened brackets
        if self.condition:
            pddl.statement += "when ({0}) (".format(self.condition)
            obr += 1
        if self.sensor_type == str(bool):
            pddl.statement += self.sensor_name if self.indicator > 0 else "not ({0})".format(self.sensor_name)
            pddl.predicates.add(self.sensor_name)  # TODO: What about other predicates employed in conditions by user??
        else:  # its numeric and not bool
            pddl.statement += "{0} ({1}) {2}".format("increase" if self.indicator > 0.0 else "decrease",
                                                     self.sensor_name, abs(self.indicator))
            pddl.functions.add(self.sensor_name)  # TODO: What about other functions employed in conditions by user??
        pddl.statement += ")" * obr  # close the brackets
        return pddl

    def get_pddl_effect_name(self):
        return get_pddl_effect_name(self.sensor_name, self.activator_name)

    def __str__(self):
        return "Effect(s:{},i:{},s_t:{},a:{},c:{})".format(str(self.sensor_name), str(self.indicator), self.sensor_type, str(self.activator_name), self.condition)


class Wish(object):

    def __init__(self, sensor_name, indicator, activator_name=''):
        self.sensor_name = sensor_name
        self.indicator = indicator
        self.activator_name = activator_name

    def get_wish_msg(self):
        """
        Get msg representation of this class
        :return: WishMsg()
        """
        msg = WishMsg()
        msg.indicator = self.indicator
        msg.sensorName = self.sensor_name
        msg.activatorName = self.activator_name
        return msg

    @staticmethod
    def from_wish_msg(wish_msg):
        """
        Create this Wish object from Wish msg type
        :param wish_msg: 
        :type wish_msg: WishMsg
        :return: Wish()
        """
        return Wish(sensor_name=wish_msg.sensorName, indicator=wish_msg.indicator, activator_name=wish_msg.activatorName)

    def get_pddl_effect_name(self):
        return get_pddl_effect_name(self.sensor_name, self.activator_name)

    def __str__(self):
        return "Wish(s:{},i:{},a:{})".format(str(self.sensor_name), str(self.indicator), str(self.activator_name))