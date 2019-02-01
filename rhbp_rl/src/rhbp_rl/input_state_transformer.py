"""
transforms values from rhbp to rl-values
@author: lehmann
"""
import numpy
from behaviour_components.sensors import EncodingConstants
from rl_config import TransitionConfig

class InputStateTransformer(object):
    """
    this class gets called in the activation algorithm and transform the rhbp components into the InputStateMessage
    """

    def __init__(self, manager):
        self._manager = manager
        self.conf = TransitionConfig()

    def calculate_reward(self):
        """
        this function calculates regarding the fulfillment and priorities of the active goals
        a reward value. 
        :return: 
        """
        reward_value = 0
        for goal in self._manager.activeGoals:
            goal_value = goal.fulfillment * goal.priority
            reward_value += goal_value
        return reward_value

    def behaviour_to_index(self, name):
        """
        gives for a given name of a behavior name the index in the behavior list
        :param name: 
        :return: 
        """
        num = 0
        for b in self._manager.behaviours:
            if b == name:
                return num
            num += 1
        return None

    def make_hot_state_encoding(self, state, num_state_space):
        """
        encodes the variables into a hot state format.
        :param state: 
        :param num_state_space: 
        :return: 
        """
        state = int(state)
        return numpy.identity(num_state_space)[state:state + 1].reshape([num_state_space, 1])

    def transform_input_values(self):
        """
        this function uses the wishes and sensors to create the input vectors
        :return: input vector
        """
        # init input array with first row of zeros
        input_array = numpy.zeros([1, 1])
        # extend input array with the sensors from conditions/behaviours
        input_array, sensor_input = self.transform_behaviours(input_array)
        # extend input array with the sensors from goals
        input_array = self.transform_goals(sensor_input, input_array)
        # cut first dummy line
        input_array = input_array[1:]
        return input_array

    def transform_behaviours(self, input_array):
        """
        extend the input array with the sensor values and wishes from the behaviours
        :param input_array: the input aray to be extended
        :return: the extended input array
        """
        use_wishes = self.conf.use_wishes
        use_true_value = self.conf.use_true_values
        # extend array with input vector from wishes
        sensor_input = {}
        # get sensor values from conditions via the behaviours
        for behaviour in self._manager.behaviours:
            # check for each sensor in the goal wishes for behaviours that have sensor effect correlations
            if use_wishes:
                for wish in behaviour.wishes:
                    wish_row = numpy.array([wish.indicator]).reshape([1, 1])
                    input_array = numpy.concatenate([input_array, wish_row])
            for sensor_value in behaviour.sensor_values:
                if not sensor_input.has_key(sensor_value.name) and use_true_value:
                    # encode or ignore the value regarding configuration in RLExtension
                    if not sensor_value.include_in_rl:
                        continue
                    if sensor_value.encoding == EncodingConstants.HOT_STATE:
                        value = self.make_hot_state_encoding(sensor_value.value, sensor_value.state_space)
                    else:
                        value = numpy.array([[sensor_value.value]])
                    sensor_input[sensor_value.name] = value
                    input_array = numpy.concatenate([input_array, value])

        return input_array, sensor_input

    def transform_goals(self, sensor_input, input_array):
        """
        transform only the goals
        :param sensor_input: saves which sensors were already included
        :param input_array: the inputs from the behaviour sensors
        :return: the updated input array. includes now the sensors of goals
        """
        use_wishes = self.conf.use_wishes
        use_true_value = self.conf.use_true_values
        # get sensors from goals
        for goal in self._manager._goals:
            for sensor_value in goal.sensor_values:
                if not sensor_input.has_key(sensor_value.name) and use_true_value:
                    # encode or ignore the value regarding configuraiton in RLExtension
                    if not sensor_value.include_in_rl:
                        continue
                    if sensor_value.encoding == EncodingConstants.HOT_STATE:
                        value = self.make_hot_state_encoding(sensor_value.value, sensor_value.state_space)
                    else:
                        value = numpy.array([[sensor_value.value]])
                    sensor_input[sensor_value.name] = value
                    input_array = numpy.concatenate([input_array, value])
            # include wishes
            if use_wishes:
                for wish in goal.wishes:
                    wish_row = numpy.array([wish.indicator]).reshape([1, 1])
                    input_array = numpy.concatenate([input_array, wish_row])

        return input_array
