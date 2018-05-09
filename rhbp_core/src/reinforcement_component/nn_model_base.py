import gym
import numpy
import numpy as np
import random
import tensorflow as tf
import matplotlib.pyplot as plt
from input_state_transformer import SensorValueTransformer
from reinforcement_learning_constants import RLConfig

class ReinforcementAlgorithmBase(object):
    def __init__(self, name):
        print("init class mnn")
        # pathes for saving the models
        self.model_path = 'models/rl-model' + name + '-1000.meta'
        self.model_folder = './models'
        self.config = RLConfig()# TODO include rospy here
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

class QLearningNeuralNetwork(ReinforcementAlgorithmBase):
    def __init__(self, name):
        super(QLearningNeuralNetwork, self).__init__(name)

    def initialize_model(self, num_inputs, num_outputs):
        """
        initializes the neural network layer. THis funciton defines how exactly the neural network looks like
        :param num_inputs: number of input values for the network
        :param num_outputs: number of output values for the network
        :return: 
        """
        tf.set_random_seed(0)
        print("initialize new model", num_inputs, num_outputs, self.name)
        # These lines establish the feed-forward part of the network used to choose actions
        self.inputs1 = tf.placeholder(shape=[1, num_inputs], dtype=tf.float32, name="inputs1")
        W1 = tf.Variable(tf.random_uniform([num_inputs, self.hl1_variables], 0, 0.01))
        if self.num_hl == 0:
            self.W2 = tf.Variable(tf.random_uniform([num_inputs, num_outputs], 0, 0.01))
            self.Qout = tf.matmul(self.inputs1, self.W2, name="Qout")
        elif self.num_hl == 1:
            self.W2 = tf.Variable(tf.random_uniform([self.hl1_variables, num_outputs], 0, 0.01))
            b1 = tf.Variable(tf.random_normal([self.hl1_variables]))
            b2 = tf.Variable(tf.random_normal([num_outputs]))
            layer = tf.add(tf.matmul(self.inputs1, W1), b1)
            self.Qout = tf.add(tf.matmul(layer, self.W2, name="Qout"), b2)
        else:
            self.W2 = tf.Variable(tf.random_uniform([self.hl1_variables, self.hl2_variables], 0, 0.01))
            b1 = tf.Variable(tf.random_normal([self.hl1_variables]))
            b2 = tf.Variable(tf.random_normal([self.hl2_variables]))

            self.W3 = tf.Variable(tf.random_uniform([self.hl2_variables, num_outputs], 0, 0.01))
            b3 = tf.Variable(tf.random_normal([num_outputs]))

            layer = tf.add(tf.matmul(self.inputs1, W1), b1)

            layer2 = tf.add(tf.matmul(layer, self.W2), b2)
            self.Qout = tf.add(tf.matmul(layer2, self.W3, name="Qout"), b3)

        # combine weights with inputs
        # choose action with highest q-value
        self.predict = tf.argmax(self.Qout, 1, name="predict")

        # Below we obtain the loss by taking the sum of squares difference between the target and prediction Q values.
        self.nextQ = tf.placeholder(shape=[1, num_outputs], dtype=tf.float32, name="nextQ")
        self.loss = tf.reduce_sum(tf.square(self.nextQ - self.Qout), name="loss")
        # loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(
        #    logits=Qout, labels=nextQ))
        self.trainer = tf.train.GradientDescentOptimizer(learning_rate=self.learning_rate_optimizer, name="trainer")
        self.updateModel = self.trainer.minimize(self.loss, name="updateModel")
        # initializes all variables so the operations can be executed
        self.init_variables = tf.global_variables_initializer()

        # 'Saver' op to save and restore all the variables
        self.saver = tf.train.Saver()

        # create session to run tf operation on
        self.sess = tf.Session()
        self.sess.run(self.init_variables)

        self.model_is_set_up = True


    def load_model(self, num_inputs, num_outputs):
        """
        loads a saved model. only gets called if the model exists
        :param num_inputs: number of input values of the network
        :param num_outputs: number of output values of the network
        :return: 
        """
        print("load model")
        # restore the session

        self.sess = tf.Session()

        # check for model with this dimensions
        self.saver = tf.train.import_meta_graph(self.model_path + "_" + num_inputs + "_" + num_outputs)

        self.saver.restore(self.sess, tf.train.latest_checkpoint(self.model_folder))

        # restore the nodes
        graph = tf.get_default_graph()
        self.Qout = graph.get_tensor_by_name("Qout:0")
        self.predict = graph.get_tensor_by_name("predict:0")
        self.inputs1 = graph.get_tensor_by_name("inputs1:0")

        self.nextQ = graph.get_tensor_by_name("nextQ:0")
        self.updateModel = graph.get_tensor_by_name("updateModel:0")

        self.model_is_set_up = True


    def train_model(self, tuple):
        """
        trains the model by inserting a tuple contaning the chosen action in a specific situation with the resulting reward.
        :param tuple: contains the last state, new state, last action and the resulting reward
        :return: 
        """
        # get fields from the input tuple
        last_state = tuple[0]
        next_state = tuple[1]
        last_action = tuple[2]
        reward = tuple[3]

        # if(not self.model_is_set_up):
        #    return

        # Obtain the Q' values by feeding the new state through our network
        Q1 = self.sess.run(self.Qout, feed_dict={self.inputs1: next_state})

        # Obtain maxQ' and set our target value for chosen action.
        maxQ1 = np.max(Q1)
        targetQ = self.allQ

        # q-learning update function for the chosen action
        targetQ[0, last_action] = reward + self.learning_rate_q_learning * maxQ1

        # Train our network using target and predicted Q values
        self.sess.run([self.updateModel], feed_dict={self.inputs1: last_state, self.nextQ: targetQ})

        # Reduce chance of random action as we train the model.
        self.epsilon = 1. / ((self.num_updates / 50) + 10)


