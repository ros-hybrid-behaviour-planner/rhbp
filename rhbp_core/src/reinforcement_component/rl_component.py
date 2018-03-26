from nn_model import ModelNeuralNetwork
from input_state_transformer import InputStateTransformer
import time
import rospy
from rhbp_core.msg import InputState, ActivationState
from rhbp_core.srv import  GetActivation,GetActivationResponse
import numpy

class RLComponent:

    def __init__(self, name="rl_component"):

        print("started rl component node",name)

        self.name = name
        self.is_model_init = False
        self.reward_list=[]
        self._getStateService = rospy.Service(name + 'GetActivation', GetActivation, self._get_activation_state_callback)
        self.model = ModelNeuralNetwork(self.name)
        self.last_state = None

        self.number_outputs = -1
        self.number_inputs = -1

    def _get_activation_state_callback(self,request):
        try:
            request=request.input_state

            self.check_if_model_is_valid(request.num_inputs,request.num_outputs)
            #print(request.input_state)
            #print(numpy.array(request.input_state))
            #print(numpy.array(request.input_state).reshape(([1,len(request.input_state)])))
            transformed_input = numpy.array(request.input_state).reshape(([1,len(request.input_state)]))
            activations = self.model.feed_forward(transformed_input)
            activations=activations.tolist()[0]
            #print(transformed_input,activations,type(activations))
            activation_state = ActivationState(**{
                "name": self.name,  # this is sent for sanity check and planner status messages only
                "activations": activations,

            })

            self.save_request(request)
            self.last_state = request.input_state

            return GetActivationResponse(activation_state)
        except Exception as e:
            print(e.message)
            return None

    def save_request(self,request):
        """
        save the old_state,new_state,action,reward tuple in a list for batch updating of the model
        :param request: 
        :return: 
        """
        if self.last_state is not None:
            reward_tuple = (self.last_state,request.input_state,request.last_action,request.reward)
            self.reward_list.append(reward_tuple)

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

        # check this in activation algo
        #if self.number_inputs < 1 or self.number_inputs < 1:
        #    print("not enough behaviors or inputs")
        #    return

        #print("num behaviors", self.number_behaviors, "num inputs", self.number_inputs, "num goals", len(self.goals))

        self.model.start_nn(num_inputs,num_outputs)

        self.reward_list=[]

        self.is_model_init = True


