"""
@author: lehmann, hrabia
"""
import rospy
from reinforcement_component.dqn_model import DQNModel
from rhbp_core.msg import ActivationState
from rhbp_core.srv import GetActivation, GetActivationResponse
import numpy


class RLComponent(object):
    def __init__(self, name):
        """
        The rl component as a class. functions as a bridge between manager and rl-algo
        :param name: name of the rl_component
        """
        # name of the rl_component
        self.name = name
        # True if the model was set up
        self.is_model_init = False
        # Service for communicating the activations
        self._getStateService = rospy.Service(name + 'GetActivation', GetActivation,
                                              self._get_activation_state_callback)
        # choose appropriate model
        self.model = DQNModel(self.name)

        # save the last state
        self.last_state = None
        # the dimensions of the model
        self.number_outputs = -1
        self.number_inputs = -1

        # current experience batch tuples (old_state,new_state,action,reward)
        self.reward_list = []

    def _get_activation_state_callback(self, request_msg):
        """
        answers the RL activation service and responds with the activations/reinforcements
        :param request_msg: GetActivation 
        :return: Service Response
        """
        input_state = request_msg.input_state
        negative_states = request_msg.negative_states
        try:

            activation_state = self.get_activation_state(input_state, negative_states)
            return GetActivationResponse(activation_state)
        except Exception as e:
            print(e)
            rospy.logerr(e.message)
            return None

    def get_activation_state(self, input_state, negative_states=None):
        """
        Determine the activation/reinforcement for the given input states, save the state (combined with last
        state for training)
        :param input_state:
        :param negative_states:
        :return:
        """
        if negative_states is None:
            negative_states = []
        try:
            self.check_if_model_is_valid(input_state.num_inputs, input_state.num_outputs)
            # save current input state and update the model
            self.save_state(input_state)
            # update the last state
            self.last_state = input_state.input_state
            for state in negative_states:
                self.save_state(state)
            # transform the input state and get activation
            transformed_input = numpy.array(input_state.input_state).reshape(([1, len(input_state.input_state)]))
            activations = self.model.feed_forward(transformed_input)
            # return the activation via the service
            activations = activations.tolist()[0]
            activation_state = ActivationState(**{
                "name": self.name,  # this is sent for sanity check and planner status messages only
                "activations": activations,
            })
            return activation_state
        except Exception as e:
            rospy.logerr(e.message)
            return None

    def save_state(self, input_state):
        """
        save the old_state,new_state,action,reward tuple in a list for batch updating of the model
        :param input_state: current state input (positive or negative)
        :type input_state: InputState
        """
        if self.last_state is None:
            return
        last = numpy.array(self.last_state).reshape(([1, len(self.last_state)]))
        new = numpy.array(input_state.input_state).reshape(([1, len(input_state.input_state)]))
        reward_tuple = (last, new, input_state.last_action, input_state.reward)

        self.reward_list.append(reward_tuple)
        self.update_model()

    def check_if_model_is_valid(self, num_inputs, num_outputs):
        """
        checks if the in-/outputs are the same as the current model has. If not 
        a new model is started
        :param num_inputs: 
        :param num_outputs: 
        :return: 
        """
        if not self.is_model_init:
            self.init_model(num_inputs, num_outputs)
        else:
            if (not self.number_outputs == num_outputs) or (not self.number_inputs == num_inputs):
                self.init_model(num_inputs, num_outputs)

    def update_model(self):
        """
        starts the training in the model for each tuple. 
        Note: the updating of the weights in the algorithm happens
        in a specified interval
        :return: 
        """
        for element in self.reward_list:
            self.model.train_model(element)
        self.reward_list = []

    def init_model(self, num_inputs, num_outputs):
        """
        inits the model with the specified parameters
        :param num_inputs: 
        :param num_outputs: 
        :return: 
        """
        self.number_inputs = num_inputs

        self.number_outputs = num_outputs

        self.last_state = None

        self.model.start_nn(num_inputs, num_outputs)

        self.reward_list = []

        self.is_model_init = True
