import matplotlib

from reinforcement_component.exploration_strategies import ExplorationStrategies
from reinforcement_component.test_environment import environment_test

matplotlib.use('agg')
from q_learning_model import QLearningNeuralNetwork
from input_state_transformer import SensorValueTransformer
import time
import rospy

from reinforcement_component.dqn_model import DQNModel
from rhbp_core.msg import InputState, ActivationState
from rhbp_core.srv import GetActivation, GetActivationResponse
import numpy
from reinforcement_component.reinforcement_learning_config import RLConstants
class RLComponent:

    def __init__(self, name, algorithm=0,pre_train=32):

        print("started rl component node",name)
        self.env_test  = environment_test()
        self.name = name
        self.is_model_init = False
        self.reward_list=[]
        self.explore = ExplorationStrategies()
        self._getStateService = rospy.Service(name + 'GetActivation', GetActivation, self._get_activation_state_callback)
        # choose appropriate model
        if algorithm == 0:
            self.model = DQNModel(self.name,pre_train)
        elif algorithm == 1:
            self.model = QLearningNeuralNetwork(self.name)
        else:
            #in case wrong number always use dqn
            self.model = DQNModel(self.name,pre_train)

        # save the last state
        self.last_state = None

        self.number_outputs = -1
        self.number_inputs = -1
        self.counter = 0.0

    def _get_activation_state_callback(self,request_msg):
        """
        answers the service and responds with the activations
        :param request: GetActivation 
        :return: 
        """
        self.counter +=1

        try:

            #print(request_msg)
            request=request_msg.input_state
            #print(request)
            #print("last",self.last_state)
            # change env input


            if self.last_state is None:
                self.last_state = [60,1]

            transformed_input = numpy.array(self.last_state).reshape(([1, len(request.input_state)]))
            if transformed_input[0][1] == 1:
                try:
                    activations = self.model.feed_forward(transformed_input)

                    #print("act",activations)
                    activations[0][0]=-1000
                    activations[0][1] = -1000
                    activations[0][4] = -1000
                    activations[0][5] = -1000
                    request.last_action = numpy.argmax(activations)
                except Exception as e:
                    print(e.message)
                    request.last_action=2
                changed,action = self.explore.e_greedy_pre_train(self.counter,2)
                if changed:
                    request.last_action = action + 2
            else:
                try:
                    activations = self.model.feed_forward(transformed_input)

                    #print("act", activations)
                    activations[0][2] = -1000
                    activations[0][3] = -1000
                    request.last_action = numpy.argmax(activations)
                except Exception as e:
                    print(e.message)
                    request.last_action = 0
                changed, action = self.explore.e_greedy_pre_train(self.counter, 4)
                if changed:
                    request.last_action = action
                    if action == 2 or action == 3:
                        request.last_action = action +2
            old_request = request
            new_t = self.env_test.get_new_tuple(self.last_state,request.last_action)
            request.input_state = new_t[0]
            request.reward = new_t[3]
            #print(request)
            #print("--------")

            # check if the model has same dimension as request and if not reinit the model
            #request.num_inputs = 134
            #request.num_outputs = 6
            #request.input_state = numpy.ones((134,1))
            self.check_if_model_is_valid(request.num_inputs,request.num_outputs)
            # save the input state in the model
            self.save_request(request)
            self.last_state = request.input_state
            #print(self.last_state,request.reward)
            negative_states = request_msg.negative_states

            if old_request.input_state[1] == 1:
                self.save_neg_state(old_request)
            else:
                self.save_neg_state_zero(old_request)
            #self.include_neg(request)
            for state in negative_states:#todo include negative ones
                #self.save_request(state)
                continue
            # transform the input state and get activation
            transformed_input = numpy.array(request.input_state).reshape(([1,len(request.input_state)]))
            activations = self.model.feed_forward(transformed_input)
            #activations = [[1,1,2,3,1,1]]
            #print(activations)
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

    def save_neg_state(self,request):
        #print("++",request)
        request.last_action = 0
        request.reward = -1
        #request.input_state=[request.input_state[0],0]
        self.save_request(request)
        request.last_action = 1
        request.reward = -1
        self.save_request(request)
        request.last_action = 4
        request.reward = -1
        self.save_request(request)
        request.last_action = 5
        request.reward = -1
        self.save_request(request)

    def save_neg_state_zero(self,request):
        #print("++",request)
        request.last_action = 2
        request.reward = -1
        #request.input_state=[request.input_state[0],0]
        self.save_request(request)
        request.last_action = 3
        request.reward = -1
        self.save_request(request)

    def get_activation_state_test(self,request,negative_states=[]):
        """
        test function which does same as the GetActivation service
        :param request: 
        :return: 
        """
        self.counter +=1
        try:
            # save currrent input state and update the model
            self.save_request(request)
            # update the last state
            self.last_state = request.input_state
            for state in negative_states:
                self.save_request(state)
            self.check_if_model_is_valid(request.num_inputs,request.num_outputs)
            transformed_input = numpy.array(request.input_state).reshape(([1,len(request.input_state)]))
            activations = self.model.feed_forward(transformed_input)
            activations=activations.tolist()[0]
            activation_state = ActivationState(**{
                "name": self.name,  # this is sent for sanity check and planner status messages only
                "activations": activations,
            })

            # updating the model with negative states has to come after the getting the activations
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
        #print( request.input_state)
        last  = numpy.array(self.last_state).reshape(([1,len(self.last_state)]))
        new = numpy.array(request.input_state).reshape(([1, len(request.input_state)]))
        reward_tuple = (last,new,request.last_action,request.reward)
        #print(reward_tuple)

        self.reward_list.append(reward_tuple)
        self.update_model()

    def include_neg(self,request):
        num = request.input_state[0]
        arr = numpy.array([[num,0]])
        arr_old = numpy.array([[self.last_state[0], 0]])
        tuple = (arr,arr,0,1)
        #print("---")
        #print(tuple)
        self.reward_list.append(tuple)
        tuple = (arr_old, arr, 1, 1)
        self.reward_list.append(tuple)
        tuple = (arr_old, arr, 4, 1)
        self.reward_list.append(tuple)
        tuple = (arr_old, arr, 5, 1)
        self.reward_list.append(tuple)
        tuple = (arr_old, arr, 2, -1)
        self.reward_list.append(tuple)
        tuple = (arr_old, arr, 3, -1)
        self.reward_list.append(tuple)

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


