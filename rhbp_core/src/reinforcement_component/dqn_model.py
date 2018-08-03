"""
class for implementing the dqn model. including savign the model,
metrics for measuring the success, the experience buffer and neural network
@author: lehmann
# inspired by awjuliani
"""


import matplotlib
import rospy

import pandas as pd

matplotlib.use('agg')
import numpy
import numpy as np
import random
import pickle
import tensorflow as tf

import matplotlib.pyplot as plt
import os
from tensorflow.contrib import slim

from nn_model_base import ReinforcementAlgorithmBase
from rl_config import NNConfig, EvaluationConfig, SavingConfig, DQNConfig, ExplorationConfig


class DQNModel(ReinforcementAlgorithmBase):
    def __init__(self, name):
        super(DQNModel, self).__init__(name)
        # Set learning parameters
        self.model_config = DQNConfig()
        self.nn_config = NNConfig()
        self.save_config = SavingConfig()
        self.evaluation = Evaluation(self.model_folder)
        self.eval_config = EvaluationConfig()
        self.exploration_config = ExplorationConfig()
        self.train_interval = self.model_config.train_interval
        self.pre_train_steps = self.model_config.pre_train  # Number of steps used before training updates begin.
        self.q_net = None
        self.target_net = None
        self.init = None
        self.targetOps = None
        # buffer class for experience learning
        self.myBuffer = ExperienceBuffer(self.model_config.buffer_size)
        self.counter = 0
        self.saver = None
        self.reward_saver = []
        self.experiment_counter = 0
        self.loss_over_time = []
        self.rewards_over_time = []

    def reset_model_values(self):
        self.counter = 0
        self.myBuffer.reset()
        self.reward_saver = []

    def initialize_model(self, num_inputs, num_outputs, load_mode=False):
        """
        initializes the neural network layer. THis function defines how exactly the neural network looks like
        :param num_inputs: number of input values for the network
        :param num_outputs: number of output values for the network
        :return: 
        """
        self.reset_model_values()
        if not load_mode:
            tf.reset_default_graph()
        # initialize two networks. q-network and target q-network
        self.q_net = QNetwork(num_inputs, num_outputs)
        self.target_net = QNetwork(num_inputs, num_outputs)
        self.init = tf.global_variables_initializer()
        # returns all variables created with trainable=True
        trainables = tf.trainable_variables()
        # create target operations
        self.targetOps = self.updateTargetGraph(trainables, self.model_config.tau)
        # buffer class for experience learning
        # self.myBuffer = experience_buffer()
        if not load_mode:
            # saver
            self.saver = tf.train.Saver()
            # with tf.Session() as self.sess:
            self.sess = tf.Session()
            # init all variables
            self.sess.run(self.init)
            # run target operations
            self.updateTarget(self.targetOps, self.sess)
            self.num_inputs = num_inputs
            self.num_outputs = num_outputs

    def feed_forward(self, input_state):
        """
        feed forwarding the input_state into the network and getting the calculated activations
        :param input_state: the input state as a vector
        :return: vector of activations
        """
        allQ = self.sess.run([self.q_net.Q_out], feed_dict={self.q_net.inputs: input_state, self.q_net.keep_per: 1.0})
        return allQ[0]

    def load_model(self, num_inputs, num_outputs):
        """
        loads a saved model. only gets called if the model exists
        :param num_inputs: number of input values of the network
        :param num_outputs: number of output values of the network
        :return: 
        """

        # restore the session
        self.sess = tf.Session()
        # check for model with this dimensions
        self.saver = tf.train.import_meta_graph(self.model_path)
        self.initialize_model(num_inputs, num_outputs, load_mode=False)
        self.saver.restore(self.sess, tf.train.latest_checkpoint(self.model_folder))
        self.load_buffer()
        rospy.loginfo("model restored")

    def load_buffer(self):
        """
        loading the experience buffer
        :return: 
        """
        size = self.conf.buffer_size
        filename = self.model_folder + "/buffer_" + str(size) + ".txt"
        try:
            with open(filename, "rb") as fp:
                buffer = pickle.load(fp)

            self.myBuffer.counter = len(buffer)
            self.myBuffer.buffer = buffer
            rospy.loginfo("experience buffer successfully loaded")
        except Exception:
            rospy.loginfo("File not found. Cannot load the experience buffer")

    def save_buffer(self):
        """
        saving the experience buffer
        :return: 
        """
        size = self.myBuffer.buffer_size
        buffer = self.myBuffer.buffer
        filename = self.model_folder + "/buffer_" + str(size) + ".txt"
        with open(filename, "wb") as fp:
            pickle.dump(buffer, fp)

    def train_model(self, tuple):
        """
        trains the model by inserting a tuple containing the chosen action in a specific situation with the resulting reward.
        :param tuple: contains the last state, new state, last action and the resulting reward
        :return: 
        """
        # save rewards
        self.rewards_over_time.append(tuple[3])
        # check if evaluation plots should be saved
        if self.counter % self.eval_config.eval_step_interval == 1:
            if self.eval_config.plot_loss:
                self.evaluation.plot_losses(self.loss_over_time)
            if self.eval_config.plot_rewards:
                self.evaluation.plot_rewards(self.rewards_over_time)
        # check if model should be saved
        if self.counter % self.save_config.steps_save == 1 and self.save_config.save:
            self.save_model()
        # save the input tuple in buffer
        transformed_tuple = np.reshape(np.array([tuple[0], tuple[2], tuple[3], tuple[1]]), [1, 4])
        self.myBuffer.add(transformed_tuple)
        # get fields from the input tuple
        self.counter += 1
        if self.counter < self.pre_train_steps or self.counter % self.train_interval != 1 \
                or self.counter > self.model_config.stop_training:
            return

        # We use Double-DQN training algorithm
        # get sample of buffer for training
        trainBatch = self.myBuffer.sample(self.model_config.batch_size)
        # feed resulting state and keep prob of 1 to predict action
        Q1 = self.sess.run(self.q_net.predict,
                           feed_dict={self.q_net.inputs: np.vstack(trainBatch[:, 3]), self.q_net.keep_per: 1.0})
        # get q-values of target network with the resulting state
        Q2 = self.sess.run(self.target_net.Q_out,
                           feed_dict={self.target_net.inputs: np.vstack(trainBatch[:, 3]),
                                      self.target_net.keep_per: 1.0})
        # multiplier to add if the episode ended
        # makes reward 0 if episode ended. simulation specific
        # target-q-values of batch for choosing prediction of q-network
        doubleQ = Q2[range(self.model_config.batch_size), Q1]  # target_q-values for the q-net predicted action
        # target q value calculation according to q-learning
        targetQ = trainBatch[:, 2] + (
            self.model_config.y * doubleQ)  # TODO add maybe here again doubleQ * endmultiplier. Nonte:works without as well
        # update the q-network model by giving the target-q-values, the input states and the chosen actions
        _, loss = self.sess.run([self.q_net.updateModel, self.q_net.loss],
                                feed_dict={self.q_net.inputs: np.vstack(trainBatch[:, 0]), self.q_net.nextQ: targetQ,
                                           self.q_net.keep_per: 1.0, self.q_net.actions: trainBatch[:, 1]})
        # save the loss function value (squared error from q and targete value)
        self.loss_over_time.append(loss)
        # update the target network
        self.updateTarget(self.targetOps, self.sess)
        # save rewards and get new state
    def updateTargetGraph(self, tfVars, tau):
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

    def updateTarget(self, op_holder, sess):
        """
        run each operation in op_holder 
        :param op_holder: 
        :param sess: 
        :return: 
        """
        for op in op_holder:
            sess.run(op)


class QNetwork(object):
    """
    this class implements the neural network. It is called for the Q-Network, but also for the target network.
    """

    def __init__(self, number_inputs, number_outputs, name="q"):
        self.nn_config = NNConfig()
        self.name = name
        # These lines establish the feed-forward part of the network used to choose actions
        # these describe the observation (input),
        self.inputs = tf.placeholder(shape=[None, number_inputs], dtype=tf.float32)
        # self.inputs = tf.cast(self.inputs,tf.float32)
        self.Temp = tf.placeholder(shape=None, dtype=tf.float32)
        self.keep_per = tf.placeholder(shape=None, dtype=tf.float32)
        # the layers that define the nn
        # one_hot_inputs = tf.one_hot(self.inputs,number_inputs,dtype=tf.float32)
        self.hidden = slim.fully_connected(self.inputs, 64, activation_fn=tf.nn.tanh,
                                           biases_initializer=None)
        # drop tensors out and scales others by probability of self.keep_per
        # self.hidden = slim.dropout(self.hidden, self.keep_per)
        # layer for computing the q_values

        self.Q_out = slim.fully_connected(self.hidden, number_outputs, activation_fn=None,
                                          biases_initializer=None)
        # prediction is highest q-value
        self.predict = tf.argmax(self.Q_out, 1)
        # compute the softmax activations.
        self.Q_dist = tf.nn.softmax(self.Q_out / self.Temp)

        # Below we obtain the loss by taking the sum of squares difference between the target and prediction Q values.
        self.actions = tf.placeholder(shape=[None], dtype=tf.int32)
        self.actions_onehot = tf.one_hot(self.actions, number_outputs, dtype=tf.float32)

        self.Q = tf.reduce_sum(tf.multiply(self.Q_out, self.actions_onehot), reduction_indices=1)
        self.nextQ = tf.placeholder(shape=[None], dtype=tf.float32)
        self.loss = tf.reduce_sum(tf.square(self.nextQ - self.Q))
        # updating the weights of the model to minimize the loss function
        if self.nn_config.use_adam_optimizer:
            trainer = tf.train.AdamOptimizer(learning_rate=self.nn_config.learning_rate_optimizer)
        else:
            trainer = tf.train.GradientDescentOptimizer(learning_rate=self.nn_config.learning_rate_optimizer)
        self.updateModel = trainer.minimize(self.loss)


# class for revisiting experiences
class ExperienceBuffer(object):
    """
    the experience buffer saves all experiences. these experiences can later be revisited for training the model.
    therefore the training batches do not correlate because not the experiences that follow on after another
    are used for training
    """

    def __init__(self, buffer_size=10000):
        self.buffer = []
        self.buffer_size = buffer_size
        self.counter = 0

    def reset(self):
        """
        reset the variables
        :return: 
        """
        self.buffer = []
        self.counter = 0

    def add(self, experience):
        """
        add a new experience
        :param experience: contains old state, new state, reward and action
        :return: 
        """
        self.counter += 1
        if len(self.buffer) + len(experience) >= self.buffer_size:
            self.buffer[0:(len(experience) + len(self.buffer)) - self.buffer_size] = []
        self.buffer.extend(experience)

    # get a random sample of the buffer
    def sample(self, size):
        """
        return a random selection of experiences with the specified size
        :param size: determines how large the sample should be
        :return: 
        """
        sample = np.reshape(np.array(random.sample(self.buffer, size)), [size, 4])
        return sample


class Evaluation(object):
    """
    for saving metrics which can measure the success of the learning process
    """

    def __init__(self, model_folder):
        self.model_folder = model_folder
        self.eval_config = EvaluationConfig()

    def plot_rewards(self, rewards_over_time):
        """
        plots and saves the rewards over time and the mean rewards
        :return: 
        """
        if not os.path.exists(self.model_folder):
            os.makedirs(self.model_folder)
        if len(rewards_over_time) == 0:
            return
        df = pd.DataFrame(numpy.array(rewards_over_time), columns=["rewards"])
        df.plot(style="o")
        plt.xlabel("steps")
        plt.savefig(self.model_folder + "/rewards_plot.png")
        plt.close()
        means = []
        batch = self.eval_config.eval_mean_size
        for i in range(0, len(rewards_over_time) / batch):
            means.append(numpy.mean(numpy.array(rewards_over_time)[batch * i:batch * (i + 1)]))
        if len(means) == 0:
            return
        df = pd.DataFrame(means, columns=["mean_rewards"])
        df.plot()
        plt.xlabel("steps")
        plt.savefig(self.model_folder + "/means_rewards_plot.png")
        plt.close()

    def plot_losses(self, loss_over_time):
        """
        plots and saves the loss error function. 
        :return: 
        """
        if not os.path.exists(self.model_folder):
            os.makedirs(self.model_folder)
        if len(loss_over_time) == 0:
            return
        df = pd.DataFrame(numpy.array(loss_over_time), columns=["loss error"])
        df.plot(legend=False)
        plt.xlabel("training steps")
        plt.ylabel("loss error")
        plt.savefig(self.model_folder + "/loss_error_plot.png")
        plt.close()
        means = []
        batch = self.eval_config.eval_mean_size
        for i in range(0, len(loss_over_time) / batch):
            means.append(numpy.mean(numpy.array(loss_over_time)[batch * i:batch * (i + 1)]))
        if len(means) == 0:
            return
        df = pd.DataFrame(means, columns=["mean loss error"])
        df.plot()
        plt.xlabel("steps")
        plt.savefig(self.model_folder + "/means_loss_error_plot.png")
        plt.close()
