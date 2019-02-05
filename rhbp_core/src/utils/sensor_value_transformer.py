"""
transforms values from rhbp to rl-values
@author: lehmann, hrabia
"""

from rhbp_core.msg import SensorValue
from behaviour_components.sensors import Sensor


class SensorValueTransformer(object):
    """
    this class is for extracting the sensor values from the conditions and creating a SensorValue message
    """

    def __init__(self):
        self.conditions = {}

    def _create_sensor_value_msg(self, sensor):
        """
        create a SensorValue msg from a RHBP core sensor
        :param sensor: a sensor
        :type sensor: Sensor
        :return: SensorValue
        """
        value = float(sensor.value)
        sensor_value = SensorValue()
        sensor_value.name = sensor.name
        sensor_value.value = value
        sensor_value.encoding = sensor.rl_extension.encoding
        sensor_value.state_space = sensor.rl_extension.state_space
        sensor_value.include_in_rl = sensor.rl_extension.include_in_rl
        return sensor_value

    def get_value_of_condition(self, cond):
        """
        gets the sensor values of a given condition
        Function recursively goes through nested conditions
        :param cond: the condition
        :return: SensorValue: a object containing necessary parameter from the sensor
        """
        try:
            return self._create_sensor_value_msg(cond.sensor)
        except Exception:
            pass
        try:
            list = []
            for c in cond.conditions:
                value = self.get_value_of_condition(c)
                if value is not None:
                    list.append(self.get_value_of_condition(c))
            return list
        except Exception:
            pass
        return None

    def get_values_of_list(self, full_list):
        """
        gets all values in list. the list can contain multiple lists
        :param full_list: 
        :return: 
        """

        new_list = []
        for ele in full_list:
            if isinstance(ele, (list,)):
                new_list.extend(self.get_values_of_list(ele))
            else:
                new_list.append(ele)
        return new_list

    def get_sensor_values(self, conditions):
        """
        gets the sensor values of a list containing multiple conditions
        :param conditions: list of conditions
        :return: list of sensorvalues
        """
        list_of_sensor_values = []
        if conditions:
            for p in conditions:
                value = self.get_value_of_condition(p)
                if isinstance(value, (list,)):
                    list_of_sensor_values.extend(self.get_values_of_list(value))
                else:
                    if value is not None:
                        list_of_sensor_values.append(value)
        return list_of_sensor_values
