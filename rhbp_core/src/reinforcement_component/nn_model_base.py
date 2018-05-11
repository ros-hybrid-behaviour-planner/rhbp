import gym
import numpy
import numpy as np
import random
import tensorflow as tf
import matplotlib.pyplot as plt
from input_state_transformer import SensorValueTransformer

class ReinforcementAlgorithmBase(object):
    def __init__(self, name):
        print("init class mnn")
        # pathes for saving the models
        self.model_path = 'models/rl-model' + name + '-1000.meta'
        self.model_folder = './models'
        self.name = name
        # tf.reset_default_graph()

        # number of hidden layers
        self.num_hl = 0

        self.hl1_variables = 19
        self.hl2_variables = 10
        # Set learning parameters
        self.learning_rate_optimizer = 0.1

        self.epsilon = 0.1
        self.learning_rate_q_learning = 0.99

        # information about current model
        self.num_updates = 0
        self.current_state = None
        self.last_action = None
        self.next_state = None

        # model variables
        self.Qout = None
        self.predict = None
        self.inputs1 = None

        self.nextQ = None
        self.updateModel = None

        self.transformer = None

        self.behaviors = []

        self.model_is_set_up = False
        self.executed_behaviours = []
        tf.set_random_seed(0)

    def start_nn(self, num_inputs, num_outputs):
        """
          calls to start the neural network. checks first if one already exists.
          :param num_inputs: 
          :param num_outputs: 
          :return: 
          """
        model_exists = self.check_if_model_exists()

        if model_exists:
            self.load_model(num_inputs, num_outputs)  # TODO check if loaded model is still valid for in and output
            # todo CHECK THAT IN THE BEGINNING MIGHT NOT ALL BEHAVIORS AND INPUTS
            # todo DIRECTLY INITIALIZED. DONT PUT MODEL AWAY
            # todo maybe save one model for each input/output number
        else:
            self.initialize_model(num_inputs, num_outputs)

    def initialize_model(self, num_inputs, num_outputs):
        raise NotImplementedError

    def check_if_model_exists(self):
        """
        check if the model exists
        :return: True if the model is saved. False otherwise
        """
        return tf.train.checkpoint_exists(self.model_folder)

    def load_model(self, num_inputs, num_outputs):
        raise NotImplementedError
    def feed_forward(self, input_state):
        """
        feed forwarding the input_state into the network and getting the calculated activations
        :param input_state: the input state as a vector
        :return: vector of activations
        """

        a, self.allQ = self.sess.run([self.predict, self.Qout], feed_dict={self.inputs1: input_state})

        return self.allQ

    def train_model(self, tuple):
        raise NotImplementedError


    def save_model(self, num_inputs, num_outputs):
        """
        saves the model 
        :param num_inputs: 
        :param num_outputs: 
        :return: 
        """
        # Save model weights to disk
        self.saver.save(self.sess, self.model_path)  # TODO save model with dim in end of name

