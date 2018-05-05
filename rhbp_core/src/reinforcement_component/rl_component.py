from nn_model import ModelNeuralNetwork
from input_state_transformer import SensorValueTransformer
import time
import rospy

from reinforcement_component.dqn_model import DQNModel
from rhbp_core.msg import InputState, ActivationState
from rhbp_core.srv import GetActivation, GetActivationResponse
import numpy
from reinforcement_component.reinforcement_learning_constants import RLConstants
class RLComponent:

    def __init__(self, name, algorithm=0):

        print("started rl component node",name)

        self.name = name
        self.is_model_init = False
        self.reward_list=[]
        self._getStateService = rospy.Service(name + 'GetActivation', GetActivation, self._get_activation_state_callback)
        if algorithm == 0:
            self.model = DQNModel(self.name)
        elif algorithm == 1:
            self.model = ModelNeuralNetwork(self.name)
        else:
            #in case wrong number always use dqn
            self.model = DQNModel(self.name)

        self.last_state = None

        self.number_outputs = -1
        self.number_inputs = -1
        self.this_run =1
        self.successfull = 0.0
        self.counter = 0.0
        self.last_100 = 0.0


    def _get_activation_state_callback(self,request):
        """
        answers the service and responds with the activations
        :param request: GetActivation 
        :return: 
        """
        self.counter +=1

        try:
            request=request.input_state
            # check if the model has same dimension as request and if not reinit the model
            self.check_if_model_is_valid(request.num_inputs,request.num_outputs)
            # save the input state in the model
            self.save_request(request)
            self.last_state = request.input_state
            # transform the input state and get activation
            transformed_input = numpy.array(request.input_state).reshape(([1,len(request.input_state)]))
            activations = self.model.feed_forward(transformed_input)
            # return the activation via the service
            activations=activations.tolist()[0]
            activation_state = ActivationState(**{
                "name": self.name,  # this is sent for sanity check and planner status messages only
                "activations": activations,
            })
            return GetActivationResponse(activation_state)
        except Exception as e:
            print(e.message)
            return None


    def get_activation_state_test(self,request):
        """
        test function which does same as the GetActivation service
        :param request: 
        :return: 
        """
        self.counter +=1
        try:
            self.save_request(request)
            self.last_state = request.input_state
            self.check_if_model_is_valid(request.num_inputs,request.num_outputs)
            transformed_input = numpy.array(request.input_state).reshape(([1,len(request.input_state)]))
            activations = self.model.feed_forward(transformed_input)
            activations=activations.tolist()[0]
            activation_state = ActivationState(**{
                "name": self.name,  # this is sent for sanity check and planner status messages only
                "activations": activations,
            })

            return activation_state
        except Exception as e:
            print(e.message)
            return None

    def save_request(self,request):
        """
        save the old_state,new_state,action,reward tuple in a list for batch updating of the model
        :param request: 
        :return: 
        """
        if self.last_state is None:
            return

        last  = numpy.array(self.last_state).reshape(([1,len(self.last_state)]))
        new = numpy.array(request.input_state).reshape(([1, len(request.input_state)]))

        reward_tuple = (last,new,request.last_action,request.reward)
        self.reward_list.append(reward_tuple)

        self.update_model()

    def check_if_model_is_valid(self,num_inputs,num_outputs):
        if not self.is_model_init:
            self.init_model(num_inputs,num_outputs)
        else:
            if (not self.number_outputs == num_outputs) or (not self.number_inputs == num_inputs):
                self.init_model(num_inputs,num_outputs)

    def update_model(self):
        for element in self.reward_list:
            self.model.train_model(element)
        self.reward_list=[]

    def init_model(self,num_inputs,num_outputs):


        self.number_inputs = num_inputs

        self.number_outputs = num_outputs

        self.model.start_nn(num_inputs,num_outputs)

        self.reward_list=[]

        self.is_model_init = True


