import gym
import numpy
import numpy as np
import random
import tensorflow as tf
import matplotlib.pyplot as plt

from tensorflow.contrib import slim

from input_state_transformer import SensorValueTransformer
from reinforcement_component.nn_model_base import ReinforcementAlgorithmBase
from reinforcement_learning_config import ExplorationConfig, NNConfig


class DQNModel(ReinforcementAlgorithmBase):
    def __init__(self, name,pre_train=32):
        super(DQNModel,self).__init__(name)
        # Set learning parameters
        self.model_config = NNConfig()

        self.exploration_config = ExplorationConfig()
        self.train_interval = self.exploration_config.train_interval
        self.pre_train_steps = self.exploration_config.pre_train  # Number of steps used before training updates begin.

        tf.set_random_seed(0)

        self.q_net = None
        self.target_net = None
        self.init = None
        self.targetOps = None
        # buffer class for experience learning
        self.myBuffer = experience_buffer(self.model_config.buffer_size)
        self.counter = 0

    def reset_model_values(self):
        self.counter = 0
        self.myBuffer.reset()

    def initialize_model(self, num_inputs, num_outputs):
        """
        initializes the neural network layer. THis funciton defines how exactly the neural network looks like
        :param num_inputs: number of input values for the network
        :param num_outputs: number of output values for the network
        :return: 
        """
        tf.set_random_seed(0)
        np.random.seed(0)
        self.reset_model_values()

        tf.reset_default_graph()
        # initialize two networks. q-network and target q-network
        self.q_net = Q_Network(num_inputs,num_outputs)
        self.target_net = Q_Network(num_inputs,num_outputs)
        self.init = tf.global_variables_initializer()
        # returns all variables created with trainable=True
        trainables = tf.trainable_variables()
        # create target operations
        self.targetOps = self.updateTargetGraph(trainables, self.model_config.tau)
        # buffer class for experience learning
        #self.myBuffer = experience_buffer()

        #with tf.Session() as self.sess:
        self.sess = tf.Session()
        # init all variables
        self.sess.run(self.init)
        # run target operations
        self.updateTarget(self.targetOps, self.sess)
        print("init mode",num_inputs,num_outputs)
    def feed_forward(self, input_state):
        """
        feed forwarding the input_state into the network and getting the calculated activations
        :param input_state: the input state as a vector
        :return: vector of activations
        """
        allQ = self.sess.run([self.q_net.Q_out], feed_dict={self.q_net.inputs: input_state,self.q_net.keep_per: 1.0})
        return allQ[0]

    def load_model(self, num_inputs, num_outputs):
        """
        loads a saved model. only gets called if the model exists
        :param num_inputs: number of input values of the network
        :param num_outputs: number of output values of the network
        :return: 
        """
        print("load model")
        # TODO add here mechanism to load a model
        self.model_is_set_up = True

    def train_model(self, tuple):
        """
        trains the model by inserting a tuple containing the chosen action in a specific situation with the resulting reward.
        :param tuple: contains the last state, new state, last action and the resulting reward
        :return: 
        """

        #save the input tuple in buffer
        #print(np.argmax(tuple[0]),np.argmax(tuple[1]),tuple[2],tuple[3])
        #print(tuple)
        self.myBuffer.add(np.reshape(np.array([tuple[0], tuple[2], tuple[3], tuple[1]]), [1, 4]))
        # get fields from the input tuple
        self.counter += 1
        if self.counter < self.pre_train_steps or self.counter % self.train_interval != 1:
            return
        #print("update")
        #print(tuple[:,3].shape)
        # We use Double-DQN training algorithm
        # get sample of buffer for training
        trainBatch = self.myBuffer.sample(self.model_config.batch_size)
        # feed resulting state and keep prob of 1 to predict action
        #print(0,trainBatch[:,3])
        Q1 = self.sess.run(self.q_net.predict, feed_dict={self.q_net.inputs: np.vstack(trainBatch[:, 3]), self.q_net.keep_per: 1.0})
        # get q-values of target network with the resulting state
        Q2 = self.sess.run(self.target_net.Q_out,
                      feed_dict={self.target_net.inputs: np.vstack(trainBatch[:, 3]), self.target_net.keep_per: 1.0})
        # multiplier to add if the episode ended
        # makes reward 0 if episode ended. simulation specific
        #end_multiplier = -(trainBatch[:, 4] - 1)
        # print(trainBatch[:,4],end_multiplier)
        # target-q-values of batch for choosing prediction of q-network
        doubleQ = Q2[range(self.model_config.batch_size), Q1]  # target_q-values for the q-net predicted action
        # target q value calculation according to q-learning
        targetQ = trainBatch[:, 2] + (self.model_config.y * doubleQ)  # TODO add maybe here again doubleQ * endmultiplier. Nonte:works without as well
        # update the q-network model by giving the target-q-values, the input states and the chosen actions
        _ = self.sess.run(self.q_net.updateModel,
                     feed_dict={self.q_net.inputs: np.vstack(trainBatch[:, 0]), self.q_net.nextQ: targetQ,
                                self.q_net.keep_per: 1.0, self.q_net.actions: trainBatch[:, 1]})
        # update the target network
        self.updateTarget(self.targetOps, self.sess)
        # save rewards and get new state

    def updateTargetGraph(self,tfVars, tau):
        """
        returns a list of operations coming from the trainable variables
        :param tfVars: 
        :param tau: 
        :return: 
        """
        total_vars = len(tfVars)
        op_holder = []
        for idx, var in enumerate(tfVars[0:total_vars / 2]):
            op_holder.append(tfVars[idx + total_vars / 2].assign(
                (var.value() * tau) + ((1 - tau) * tfVars[idx + total_vars / 2].value())))
        return op_holder

    def updateTarget(self,op_holder, sess):
        """
        run each operation in op_holder 
        :param op_holder: 
        :param sess: 
        :return: 
        """
        for op in op_holder:
            sess.run(op)


 #Implementing the network itself
class Q_Network():
    def __init__(self,number_inputs,number_outputs):
        tf.set_random_seed(0)
        # These lines establish the feed-forward part of the network used to choose actions
        # these describe the observation (input),
        self.inputs = tf.placeholder(shape=[None, number_inputs], dtype=tf.float32)
        #self.inputs = tf.cast(self.inputs,tf.float32)
        self.Temp = tf.placeholder(shape=None, dtype=tf.float32)
        self.keep_per = tf.placeholder(shape=None, dtype=tf.float32)

        # the layers that define the nn
        #one_hot_inputs = tf.one_hot(self.inputs,number_inputs,dtype=tf.float32)
        hidden = slim.fully_connected(self.inputs, 64, activation_fn=tf.nn.tanh, biases_initializer=None)
        # drop tensors out and scales others by probability of self.keep_per
        hidden = slim.dropout(hidden, self.keep_per)
        # layer for computing the q_values
        self.Q_out = slim.fully_connected(hidden, number_outputs, activation_fn=None, biases_initializer=None)
        # prediction is highest q-value
        self.predict = tf.argmax(self.Q_out, 1)
        # compute the softmax activations.
        self.Q_dist = tf.nn.softmax(self.Q_out / self.Temp)

        # Below we obtain the loss by taking the sum of squares difference between the target and prediction Q values.
        self.actions = tf.placeholder(shape=[None], dtype=tf.int32)
        self.actions_onehot = tf.one_hot(self.actions, number_outputs, dtype=tf.float32)

        self.Q = tf.reduce_sum(tf.multiply(self.Q_out, self.actions_onehot), reduction_indices=1)
        #tf.mul
        self.nextQ = tf.placeholder(shape=[None], dtype=tf.float32)
        loss = tf.reduce_sum(tf.square(self.nextQ - self.Q))
        # updating the weights of the model to minimize the loss function
        trainer = tf.train.GradientDescentOptimizer(learning_rate=0.0005)
        self.updateModel = trainer.minimize(loss)


# class for revisiting experiences
class experience_buffer():
    def __init__(self, buffer_size=10000):
        self.buffer = []
        self.buffer_size = buffer_size
        self.counter = 0
        self.print_steps = False
        random.seed(0)
    # add a new experience
    def reset(self):
        self.buffer=[]
        random.seed(0)
        self.counter=0
    def add(self, experience):
        if  self.print_steps:
            print(self.counter, np.argmax(experience[0, 0]), np.argmax(experience[0, 3]), experience[0, 1], experience[0, 2])
        self.counter += 1
        if len(self.buffer) + len(experience) >= self.buffer_size:
            self.buffer[0:(len(experience) + len(self.buffer)) - self.buffer_size] = []
        self.buffer.extend(experience)
    # get a random sample of the buffer

    def sample(self, size):
        sample = np.reshape(np.array(random.sample(self.buffer, size)), [size, 4])
        if  self.print_steps:
            print("train", np.argmax(sample[10][0]), np.argmax(sample[10][3]), sample[10][1], sample[10][2])
        return sample

