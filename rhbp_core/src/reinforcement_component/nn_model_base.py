import tensorflow as tf
import os

from reinforcement_component.rl_config import NNConfig


class ReinforcementAlgorithmBase(object):
    def __init__(self, name):
        # paths for saving the models
        self.conf = NNConfig()

        self.model_path = self.conf.model_path + name + '-1000.meta'
        self.model_folder = self.conf.model_directory
        self.name = name
        # dimensions
        self.num_inputs = 0
        self.num_outputs = 0
        self.num_hl = 0
        # model variables
        self.Qout = None
        self.predict = None
        self.inputs1 = None

        self.nextQ = None
        self.updateModel = None

        self.model_is_set_up = False

    def start_nn(self, num_inputs, num_outputs):
        """
          calls to start the neural network. checks first if one already exists.
          :param num_inputs: 
          :param num_outputs: 
          :return: 
          """
        model_exists = self.check_if_model_exists(num_inputs, num_outputs)

        if model_exists:
            self.load_model(num_inputs, num_outputs)  # TODO check if loaded model is still valid for in and output
            # todo CHECK THAT IN THE BEGINNING MIGHT NOT ALL BEHAVIORS AND INPUTS
            # todo DIRECTLY INITIALIZED. DONT PUT MODEL AWAY
            # todo maybe save one model for each input/output number
        else:
            self.initialize_model(num_inputs, num_outputs)

    def initialize_model(self, num_inputs, num_outputs):
        raise NotImplementedError

    def check_if_model_exists(self, num_inputs, num_outputs):
        """
        check if the model exists
        :return: True if the model is saved. False otherwise
        """
        self.model_path = 'models/' + str(num_inputs) + '/' + str(num_outputs) + '/rl-model' + self.name + "_" + str(
            num_inputs) + "_" + str(num_outputs) + '-1000.meta'
        self.model_folder = './models/' + str(num_inputs) + '/' + str(num_outputs)
        try:
            model_exists = tf.train.checkpoint_exists(self.model_folder)
            return model_exists
        except Exception as e:
            return False

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
        if not self.conf.save:
            return

        if not os.path.exists(self.model_folder):
            os.makedirs(self.model_folder)

        # Save model weights to disk
        self.saver.save(self.sess, self.model_path)
