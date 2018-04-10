from nn_model import ModelNeuralNetwork
from input_state_transformer import SensorValueTransformer
import time
import rospy
from rhbp_core.msg import InputState, ActivationState
from rhbp_core.srv import  GetActivation,GetActivationResponse
import numpy

class RLComponent:

    def __init__(self, name):

        print("started rl component node",name)

        self.name = name
        self.is_model_init = False
        self.reward_list=[]
        self._getStateService = rospy.Service(name + 'GetActivation', GetActivation, self._get_activation_state_callback)
        self.model = ModelNeuralNetwork(self.name)
        self.last_state = None

        self.number_outputs = -1
        self.number_inputs = -1
        self.this_run =1
        self.successfull = 0.0
        self.counter = 0.0
        self.last_100 = 0.0

    def _get_activation_state_callback(self,request):
        self.counter +=1

        try:
            request=request.input_state

            self.save_request(request)
            self.last_state = request.input_state

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


            #print(numpy.argmax(request.input_state),numpy.argmax(activations),numpy.round(activations,5),
            #      self.this_run,self.successfull/self.this_run,self.last_100/100.0)
            return GetActivationResponse(activation_state)
        except Exception as e:
            print(e.message)
            return None


    def get_activation_state_test(self,request):
        self.counter +=1
        #if self.counter % 4 == 0:
        #self.update_model()
        try:
            self.save_request(request)
            self.last_state = request.input_state
            #request=request.input_state
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


            #print(numpy.argmax(request.input_state),numpy.argmax(activations),numpy.round(activations,5),
            #      self.this_run,self.successfull/self.this_run,self.last_100/100.0)
            return activation_state
        except Exception as e:
            print(e.message)
            return None


    def get_array(self,s):
        return numpy.identity(16)[s:s + 1]

    def save_request(self,request):
        """
        save the old_state,new_state,action,reward tuple in a list for batch updating of the model
        :param request: 
        :return: 
        """
        #if self.is_terminal(numpy.argmax(self.last_state)) :
        #    return
        if self.last_state is not None:

            last  = numpy.array(self.last_state).reshape(([1,len(self.last_state)]))
            if int(request.reward) == 10:
                #print("------------------------------------------")
                 new = [self.get_array(15).tolist()[0]]
            else:
                new = numpy.array(request.input_state).reshape(([1, len(request.input_state)]))
            new = numpy.array(request.input_state).reshape(([1, len(request.input_state)]))
            #new = numpy.array(request.resulting_state).reshape(([1, len(request.resulting_state)]))

            reward_tuple = (last,new,request.last_action,request.reward)
            #print(numpy.argmax(last), numpy.argmax(new), request.last_action, request.reward)
            self.reward_list.append(reward_tuple)
            self.update_model()
            print(numpy.argmax(last),numpy.argmax(new),request.last_action,request.reward)
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


