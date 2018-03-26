from nn_model import ModelNeuralNetwork
from input_state_transformer import InputStateTransformer
import time
import rospy
from rhbp_core.msg import InputState, ActivationState
from rhbp_core.srv import  GetActivation,GetActivationResponse

class RLComponent:

    def __init__(self, name="rl_component"):

        print("started rl component node",name)

        self.name = name
        self.is_model_init = False
        self.reward_list=[]
        self._getStateService = rospy.Service(name + 'GetActivation', GetActivation, self._get_activation_state_callback)
        self.model = ModelNeuralNetwork(self.name)
        self.last_state = None


    def _get_activation_state_callback(self,request ):
        try:
            # setup model TODO
            self.check_if_model_is_valid(request.num_input,request.num_output)
            activations = self.get_rl_activation(request.input_state)

            activation_state = ActivationState(**{
                "name": self.name,  # this is sent for sanity check and planner status messages only
                "activation": activations,

            })
            self.save_request(request)
            self.last_state = request.input_state
            return GetActivationResponse(activation_state)
        except Exception:
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

    def check_if_model_is_valid(self,num_input,num_output):
        if not self.is_model_init:
            self.init_model(num_input,num_output)
        else:
            if (not self.number_outputs == num_output) or (not self.number_inputs == num_input):
                self.init_model(num_input,num_output)

    def init_model(self,num_inputs,num_outputs):

        self.number_inputs = num_inputs

        self.number_outputs = num_outputs

        if self.number_behaviors < 1 or self.number_inputs < 1:
            print("not enough behaviors or inputs")
            return

        #print("num behaviors", self.number_behaviors, "num inputs", self.number_inputs, "num goals", len(self.goals))

        self.model.start_nn(num_inputs,num_outputs)


        self.is_model_init = True


if __name__ == '__main__':
    try:
        print("trying ot start")
        rospy.init_node('rl_node', anonymous=True)
        rl_component = RLComponent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")