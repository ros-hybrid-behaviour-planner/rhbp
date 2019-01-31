"""
base class for implementing a reinforcement learning algorithm with a neural network
@author: lehmann
"""
import rospy
import tensorflow as tf
import os
from rl_config import SavingConfig


class ReinforcementAlgorithmBase(object):
    def __init__(self, name):
        # paths for saving the models
        self.save_conf = SavingConfig()
        self.model_path = self.save_conf.model_path + name + '-1000.meta'
        self.model_folder = self.save_conf.model_directory
        self.name = name
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
            try:
                self.load_model(num_inputs, num_outputs)
            except Exception as e:
                rospy.logerr("Failed loading model, initialising a new one. Error: %s", e)
                self.initialize_model(num_inputs, num_outputs)
        else:
            self.initialize_model(num_inputs, num_outputs)

    def initialize_model(self, num_inputs, num_outputs):
        raise NotImplementedError

    def check_if_model_exists(self, num_inputs, num_outputs):
        """
        check if the model exists
        :return: True if the model is saved. False otherwise
        """
        # TODO using just the number of inputs/outputs can also lead to weird results?! It would be better to use proper identifiers

        self.model_folder = 'models/' + str(num_inputs) + '/' + str(num_outputs) + '/' + self.name
        self.model_path = self.model_folder + '/rl-model' + self.name + "_" + str(num_inputs) + "_" + str(num_outputs) \
                          + '-1000.meta'  # TODO Why this postfix?

        if not os.path.exists(self.model_folder):
            os.makedirs(self.model_folder)
        if not self.save_conf.load:
            return False
        try:
            model_exists = tf.train.checkpoint_exists(self.model_path)
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

    def save_model(self):
        """
        saves the model 
        :return: 
        """
        if not self.save_conf.save:
            return

        if not os.path.exists(self.model_folder):
            os.makedirs(self.model_folder)

        # Save model weights to disk
        # cut out last 5 character as they would be twice
        self.saver.save(self.sess, self.model_path[0:-5])

        if self.save_conf.save_buffer:
            self.save_buffer()

        rospy.loginfo("model saved")

    def save_buffer(self):
        raise NotImplementedError

    def load_buffer(self):
        raise NotImplementedError
