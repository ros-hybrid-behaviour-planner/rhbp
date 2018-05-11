import gym
import numpy
import numpy as np
import random
import tensorflow as tf
import matplotlib.pyplot as plt
from input_state_transformer import SensorValueTransformer
from reinforcement_component.nn_model_base import ReinforcementAlgorithmBase
from reinforcement_learning_config import RLConfig

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


