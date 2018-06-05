import matplotlib
matplotlib.use('agg')
import gym
import numpy
import numpy as np
import random

import pandas
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
        #self.myBuffer.fill_buffer(self.model_config.buffer_size)#todo delet elater
        self.counter = 0
        self.saver = None

    def reset_model_values(self):
        self.counter = 0
        self.myBuffer.reset()

    def initialize_model(self, num_inputs, num_outputs,load_mode=False):
        """
        initializes the neural network layer. THis function defines how exactly the neural network looks like
        :param num_inputs: number of input values for the network
        :param num_outputs: number of output values for the network
        :return: 
        """
        tf.set_random_seed(0)
        np.random.seed(0)
        self.reset_model_values()
        if not load_mode:
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
        if not load_mode:
            # saver
            self.saver = tf.train.Saver()
            #with tf.Session() as self.sess:
            self.sess = tf.Session()
            # init all variables
            self.sess.run(self.init)
            # run target operations
            self.updateTarget(self.targetOps, self.sess)
            print("init mode",num_inputs,num_outputs)
            self.num_inputs = num_inputs
            self.num_outputs = num_outputs

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

        # restore the session
        self.sess = tf.Session()
        print(1,self.model_path)
        # check for model with this dimensions
        self.saver = tf.train.import_meta_graph(self.model_path)
        print("2",self.model_folder)
        self.saver.restore(self.sess, tf.train.latest_checkpoint(self.model_folder))
        print(3)
        # restore the nodes
        graph = tf.get_default_graph()

        self.initialize_model(num_inputs,num_outputs,load_mode=True)

        self.q_net.load(graph)

        self.target_net.load(graph)

        self.updateTarget(self.targetOps, self.sess)

        self.sess.run(tf.global_variables_initializer())
        self.model_is_set_up = True

    def print_weights_bool(self):

        num=self.counter/self.model_config.steps_prints
        names = ["turn_to_ball","search_goal","dribble_ball","shoot","search_ball","go_to_ball"]
        input_not_kickable = numpy.array([[1,1,1,10,60]])
        input_kick_ball = numpy.array([[1, 1, 1, 0.5,40, 0,20,0]])
        #input_ball_not_seen = numpy.array([[1, 1, 1, 0.5,20, 0,20,0]])
        #input_goal_not_seen = numpy.array([[1, 0, 1, 0.5,20, 0,20,0]])
        input_not_playable = numpy.array([[0, 1, 1, 10,60]])

        res_input_not_kickable = self.feed_forward(input_not_kickable)[0]
        res_input_kick_ball = self.feed_forward(input_kick_ball)[0]
        #res_input_ball_not_seen = self.feed_forward(input_ball_not_seen)[0]
        #res_input_goal_not_seen = self.feed_forward(input_goal_not_seen)[0]
        res_input_not_playable = self.feed_forward(input_not_playable)[0]

        df = pandas.DataFrame(res_input_not_kickable,columns=["activations"])
        df["names"]=pandas.DataFrame(names)
        df.plot(x="names", y="activations")
        path = "figures/actions/not_kickable_comp_" + str(num) + ".png"
        plt.savefig(path)
        plt.close()

        df = pandas.DataFrame(res_input_kick_ball,columns=["activations"])
        df["names"]=pandas.DataFrame(names)
        df.plot(x="names", y="activations")
        path = "figures/actions/kick_ball_comp_" + str(num) + ".png"
        plt.savefig(path)
        plt.close()
        """    
        df = pandas.DataFrame(res_input_ball_not_seen,columns=["activations"])
        df["names"]=pandas.DataFrame(names)
        df.plot(x="names", y="activations")
        path = "figures/actions/ball_not_seen_comp_" + str(num) + ".png"
        plt.savefig(path)
        plt.close()

        df = pandas.DataFrame(res_input_goal_not_seen,columns=["activations"])
        df["names"]=pandas.DataFrame(names)
        df.plot(x="names", y="activations")
        path = "figures/actions/goal_not_seen_comp_" + str(num) + ".png"
        plt.savefig(path)
        plt.close()
        """
        df = pandas.DataFrame(res_input_not_playable,columns=["activations"])
        df["names"]=pandas.DataFrame(names)
        df.plot(x="names", y="activations")
        path = "figures/actions/not_playable_comp_" + str(num) + ".png"
        plt.savefig(path)
        plt.close()

    def print_weights_discrete(self):

        nummer=self.counter/self.model_config.steps_prints
        array = numpy.array([[1,1,1,1]])
        input1 = numpy.array([[1, 1, 1, 1, 1, 0, 0, 0, 0, 0,0]])
        input2 = numpy.array([[1, 1, 1, 1, 0, 1, 0, 0, 0, 0,0]])
        input3 = numpy.array([[1, 1, 1, 1, 0, 0, 1, 0, 0, 0,0]])
        input4 = numpy.array([[1, 1, 1, 1, 0, 0, 0, 1, 0, 0,0]])
        input5 = numpy.array([[1, 1, 1, 1, 0, 0, 0, 0, 1, 0,0]])
        input6 = numpy.array([[1, 1, 1, 1, 0, 0, 0, 0, 0, 1,0]])
        inputs = [input1,input2,input3,input4,input5,input6]
        num=0
        for input in inputs:
            results = self.feed_forward(input)
            dribble = results[0][2]
            shoot = results[0][3]
            line = numpy.array([[num,dir,dribble,shoot]])
            array = numpy.concatenate( (array,line),axis=0 )
            num += 1
        #df = pandas.DataFrame(array[1:],columns=["dist","dir","dribble","shoot"])
        #df.plot(x="dist",y=["dribble","shoot"])
        #plt.show()
        plt.plot(array[1:,0],array[1:,2],label="dribble")
        plt.plot(array[1:, 0], array[1:, 3],label="shoot")
        plt.xlabel("distance")
        plt.ylabel("activation")
        plt.legend()
        #plt.show()
        path="figures/discrete/d_s_dist_comp_"+str(nummer)+".png"
        plt.show()
        plt.savefig(path)
        plt.close()

    def print_weights(self):
        num=self.counter/self.model_config.steps_prints
        array = numpy.array([[0,0,0,0,0,0]])
        for dist in numpy.arange(60,0,-0.5):
                dir = 0
                input = numpy.array([[1,1,1,0.5,dist]])
                results = self.feed_forward(input)
                dribble = results[0][2]
                shoot = results[0][3]
                turn = results[0][0]
                goal = results[0][1]
                ball = results[0][4]
                goto = results[0][5]
                line = numpy.array([[dist,dir,dribble,shoot,turn,goto]])
                array = numpy.concatenate( (array,line),axis=0 )
        #df = pandas.DataFrame(array[1:],columns=["dist","dir","dribble","shoot"])
        #df.plot(x="dist",y=["dribble","shoot"])
        #plt.show()
        plt.plot(array[1:,0],array[1:,2],label="dribble")
        plt.plot(array[1:, 0], array[1:, 3],label="shoot")
        plt.plot(array[1:, 0], array[1:, 4], label="turn")
        plt.plot(array[1:, 0], array[1:, 5], label="goto")
        plt.xlabel("distance in degrees")
        plt.ylabel("activation")
        plt.legend()
        #plt.show()
        path="figures/dist/d_s_dist_comp_"+str(num)+".png"
        plt.show()
        plt.savefig(path)
        plt.close()
        return
        array = numpy.array([[0, 0, 0, 0]])
        for dir in numpy.arange(-45, 45, 1):
            dist=25
            input = numpy.array([[1,1,1,0.1,dist, dir,dist-0.1,0]])
            results = self.feed_forward(input)
            dribble = results[0][2]
            shoot = results[0][3]
            line = numpy.array([[dist, dir, dribble, shoot]])
            array = numpy.concatenate((array, line), axis=0)

        plt.plot(array[1:, 1], array[1:, 2],label="dribble")
        plt.plot(array[1:, 1], array[1:, 3],label="shoot")
        plt.xlabel("direction in degrees")
        plt.ylabel("activation")
        plt.legend()
        # plt.show()
        path = "figures/dir/d_s_dir_comp_" + str(num) + ".png"
        plt.savefig(path)
        plt.close()

    def train_model(self, tuple):
        """
        trains the model by inserting a tuple containing the chosen action in a specific situation with the resulting reward.
        :param tuple: contains the last state, new state, last action and the resulting reward
        :return: 
        """
        #todo COUNTER DOES NOT ONLY COUNT STEPS BUT ALSO  negative steps included
        if self.counter % self.model_config.steps_save == 1 and self.model_config.save:
            self.save_model(self.num_inputs,self.num_outputs)
        #save the input tuple in buffer
        #print(np.argmax(tuple[0]),np.argmax(tuple[1]),tuple[2],tuple[3])
        #print(tuple,self.counter)
        if self.counter % self.model_config.steps_prints == 1 and self.model_config.print_model:
            #self.print_weights_discrete()
            self.print_weights_bool()
            self.print_weights()
        #if self.counter > self.exploration_config.stop_training:
        #    return
        #array = tuple[0][0]
        #if array[0]==1 and array[1]==1 and array[2]==1 and array[3]==1: #TODO save only tuples that are kciking ball
        #self.myBuffer.add(np.reshape(np.array([tuple[0], tuple[2], tuple[3], tuple[1]]), [1, 4]))
        # get fields from the input tuple
        self.counter += 1
        #todo/problem with interval = 5 got trained two times per step because one step is 5 tuples(negative included)
        if self.counter < self.pre_train_steps or self.counter % self.train_interval != 1 \
                or self.counter > self.exploration_config.stop_training:
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
    def __init__(self,number_inputs,number_outputs,name="q"):
        self.name = name
        tf.set_random_seed(0)
        # These lines establish the feed-forward part of the network used to choose actions
        # these describe the observation (input),
        self.inputs = tf.placeholder(shape=[None, number_inputs], dtype=tf.float32)
        #self.inputs = tf.cast(self.inputs,tf.float32)
        self.Temp = tf.placeholder(shape=None, dtype=tf.float32)
        self.keep_per = tf.placeholder(shape=None, dtype=tf.float32)

        # the layers that define the nn
        #one_hot_inputs = tf.one_hot(self.inputs,number_inputs,dtype=tf.float32)
        self.hidden = slim.fully_connected(self.inputs, 64, activation_fn=tf.nn.tanh, biases_initializer=tf.random_uniform_initializer())
        #self.hidden2 = slim.fully_connected(self.hidden, 128, activation_fn=tf.nn.tanh, biases_initializer=tf.random_uniform_initializer())

        #self.hidden3 = slim.fully_connected(self.hidden2, 64, activation_fn=tf.nn.tanh, biases_initializer=tf.random_uniform_initializer())

        #self.hidden4 = slim.fully_connected(self.hidden3, 96, activation_fn=tf.nn.tanh, biases_initializer=None)

        #self.hidden5 = slim.fully_connected(self.hidden4, 64, activation_fn=tf.nn.tanh, biases_initializer=None)

        #self.hidden6 = slim.fully_connected(self.hidden5, 32, activation_fn=tf.nn.tanh, biases_initializer=None)
        # drop tensors out and scales others by probability of self.keep_per
        self.hidden = slim.dropout(self.hidden, self.keep_per)
        # layer for computing the q_values
        self.Q_out = slim.fully_connected(self.hidden, number_outputs, activation_fn=None, biases_initializer=tf.random_uniform_initializer())
        # prediction is highest q-value
        self.predict = tf.argmax(self.Q_out, 1)
        # compute the softmax activations.
        self.Q_dist = tf.nn.softmax(self.Q_out / self.Temp)

        # Below we obtain the loss by taking the sum of squares difference between the target and prediction Q values.
        self.actions = tf.placeholder(shape=[None], dtype=tf.int32)
        self.actions_onehot = tf.one_hot(self.actions, number_outputs, dtype=tf.float32) # TODO here could also multiple actions be included

        self.Q = tf.reduce_sum(tf.multiply(self.Q_out, self.actions_onehot), reduction_indices=1)
        #tf.mul
        self.nextQ = tf.placeholder(shape=[None], dtype=tf.float32)
        loss = tf.reduce_sum(tf.square(self.nextQ - self.Q))
        # updating the weights of the model to minimize the loss function
        trainer = tf.train.GradientDescentOptimizer(learning_rate=0.0005)
        self.updateModel = trainer.minimize(loss)

    def load(self,graph):
        name_hidden = "hidden_"+self.name+":0"
        name_q_out = "q_out" + self.name+":0"
        self.hidden = graph.get_tensor_by_name(name_hidden)
        self.Q_out = graph.get_tensor_by_name(name_q_out)

# class for revisiting experiences
class experience_buffer():
    def __init__(self, buffer_size=10000):
        self.buffer = []
        self.buffer_size = buffer_size
        self.counter = 0
        self.print_steps = False
        random.seed(0)
        self.sim_counter = 0
        self.dict_sequ = {"ds":(0,0),"ss":(0,0),"ds":(0,0),"ddd":(0,0),"dds":(0,0),"sd":(0,0)}
        self.sequ = ""
    # add a new experience
    def reset(self):
        self.buffer=[]
        random.seed(0)
        self.counter=0
        self.fill_buffer(self.buffer_size/100.0)#TODO delet elate
        print("results",self.dict_sequ)
    def add(self, experience,is_negative=False):
        if not is_negative:
            self.add_negative_experience(experience)
        experience = np.reshape(np.array([experience[0], experience[2], experience[3], experience[1]]), [1, 4])
        if not is_negative:
            #print(experience,self.sim_counter)
            self.sim_counter+=1
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

    def is_chance(self,chance):
        value=np.random.rand(1)
        if value <= chance:
            return True
        return False

    def random_dist(self):
        num = numpy.random.randint(low=0,high=5)
        dist0 = 0
        dist1 = 0
        dist2 = 0
        dist3 = 0
        dist4 = 0
        dist5 = 0
        if num==0:
            dist0 = 1
        if num==1:
            dist1 = 1
        if num==2:
            dist2 = 1
        if num==3:
            dist3 = 1
        if num==4:
            dist4 = 1
        if num==5:
            dist5 = 1
        return dist0,dist1,dist2,dist3,dist4,dist5,num

    def next_dist(self,num):
        dist0 = 0
        dist1 = 0
        dist2 = 0
        dist3 = 0
        dist4 = 0
        dist5 = 0
        if num == 0:
            dist1 = 1
        if num == 1:
            dist2 = 1
        if num == 2:
            dist3 = 1
        if num == 3:
            dist4 = 1
        if num == 4:
            dist5 = 1
        if num == 5:
            dist5 = 1
        return dist0, dist1, dist2, dist3, dist4, dist5, num+1

    def reward_from_dist(self,num):
        return 4+num*1.5
    def add_expected_tuple(self):
        action = numpy.random.randint(low=0,high=5)

        play = 1
        ball = 1
        goal = 1
        kick = 1
        speed = 0
        dist0 = 0
        dist1 = 0
        dist2 = 0
        dist3 = 0
        dist4 = 0
        dist5 = 1

        if action == 0: #turn_to_ball
            play=0
            kick=0

            is_state = [play,ball,goal,kick,dist0,dist1,dist2,dist3,dist4,dist5,speed]
            reward = 104
            if self.is_chance(0.1):
                next_state =[1,ball,goal,kick,dist0,dist1,dist2,dist3,dist4,dist5,speed]
                reward=4
            else:
                next_state =[play,ball,goal,kick,dist0,dist1,dist2,dist3,dist4,dist5,speed]

        if action == 1:#search_goal
            goal=0
            dist0, dist1, dist2, dist3, dist4, dist5,num = self.random_dist()
            is_state = [play,ball,goal,kick,dist0,dist1,dist2,dist3,dist4,dist5,speed]

            reward = self.reward_from_dist(num)
            if self.is_chance(0.3):
                next_state =[play,ball,1,kick,dist0,dist1,dist2,dist3,dist4,dist5,speed]
            else:
                next_state =[play,ball,goal,kick,dist0,dist1,dist2,dist3,dist4,dist5,speed]

        if action == 2:  # dribble
            speed=10
            dist0, dist1, dist2, dist3, dist4, dist5, num = self.random_dist()
            is_state = [play, ball, goal, kick, dist0, dist1, dist2, dist3, dist4, dist5,0]

            reward = self.reward_from_dist(num)
            next_state = [play, ball, goal, 0, dist0, dist1, dist2, dist3, dist4, dist5,speed]

        if action == 3:  # shoot
            speed=50
            dist0, dist1, dist2, dist3, dist4, dist5, num = self.random_dist()
            is_state = [play, ball, goal, kick, dist0, dist1, dist2, dist3, dist4, dist5,0]

            reward = self.reward_from_dist(num)
            next_state = [play, ball, goal, 0, dist0, dist1, dist2, dist3, dist4, dist5,speed]

        if action == 4:  # search_ball
            ball = 0
            kick=0
            if self.is_chance(0.5):
                goal=0
            dist0, dist1, dist2, dist3, dist4, dist5, num = self.random_dist()
            is_state = [play, ball, goal, kick, dist0, dist1, dist2, dist3, dist4, dist5,speed]
            reward = self.reward_from_dist(num)
            if self.is_chance(0.3):
                next_state = [play, 1,goal, kick, dist0, dist1, dist2, dist3, dist4, dist5,speed]
            else:
                next_state = [play, ball, goal, kick, dist0, dist1, dist2, dist3, dist4, dist5,speed]

        if action == 5:  # got_to_ball
            kick = 0
            if self.is_chance(0.5):
                goal=0
            dist0, dist1, dist2, dist3, dist4, dist5, num = self.random_dist()
            is_state = [play, ball, goal, kick, dist0, dist1, dist2, dist3, dist4, dist5,speed]
            if self.is_chance(0.2):
                dist0, dist1, dist2, dist3, dist4, dist5, num = self.next_dist(num)
            reward = self.reward_from_dist(num)
            if self.is_chance(0.01):
                next_state = [0, 1, 1, 0, 0, 0, 0, 0, 0, 1]
                reward = 104
            elif self.is_chance(0.1):
                next_state = [play, ball, goal, 1, dist0, dist1, dist2, dist3, dist4, dist5,speed]
            else:
                next_state = [play, ball, goal, kick, dist0, dist1, dist2, dist3, dist4, dist5,speed]

        return is_state,next_state,action,reward

    def fill_buffer_old(self,num):
        for i in range(0,num):
            tuple=self.add_expected_tuple()
            sample = np.reshape(np.array([tuple[0], tuple[2], tuple[3], tuple[1]]), [1, 4])
            self.add(sample)

    def fill_buffer(self,num):
        for i in range(0,int(num)):
            self.add_trial()
            print(" ")
            print(" ")
            #print(self.sequ)
            dict  =self.dict_sequ
            value = dict.get(self.sequ)
            newvalue=(value[0]+self.sim_counter,value[1]+1)
            self.dict_sequ[self.sequ]=newvalue

            self.sim_counter=0
            self.sequ=""
    def get_tuple_go_to_ball_start(self,ball,goal,num,max_moves):
        ball = ball - (10 / max_moves)
        goal = goal - (10 / max_moves)
        return ball,goal

    def get_tuple_go_to_ball_shoot(self,ball,goal,num,max_moves,speed):
        if num<=2:
            ball = ball +9.5
            speed -= 20
        elif num ==3:
            ball = ball + 5.0
            speed -= 10
        else:
            ball -= 0.5
            speed = 0
        goal = goal - 0.5
        if speed <= 0:
            speed = 0
        return ball,goal,speed

    def get_tuple_go_to_ball_dribble(self, ball, goal, num, max_moves,speed):
        if num <= 2:
            ball = ball + 4.5
            speed = speed - 10
        elif num == 3:
            ball = ball + 4.0
        else:
            ball -= 0.5
            speed = 0
        if speed <= 0:
            speed = 0
        goal = goal - 0.5

        return ball, goal,speed

    def add_negative_experience(self,experience):
        #return
        for i in range(0,6):
            if i == experience[1]:
                continue
            else:
                self.add( [experience[0],experience[1],i,experience[3]],True)

    def add_trial(self):
        step_reward = -1
        dist_ball = 10
        dist_goal = 60
        not_play_t = [0,1,1,dist_ball,dist_goal]
        reward_not = 3
        experience_stay_not_play = [not_play_t, not_play_t, 0, reward_not]
        num_wait_before_play = 75
        for i in range(0,num_wait_before_play-1):
            self.add(experience_stay_not_play)

        after_not_play_t = [1,1,1,dist_ball,dist_goal]
        experience_after_not_play = [not_play_t,after_not_play_t,0,-1]
        self.add(experience_after_not_play)
        old_tuple = after_not_play_t
        num_go_to_first_ball = 20.0
        for i in range(0,int(num_go_to_first_ball-1)):
            dist_ball,dist_goal = self.get_tuple_go_to_ball_start(dist_ball,dist_goal,i,num_go_to_first_ball)
            tuple = [1,1,1,dist_ball,dist_goal]
            experience = [old_tuple,tuple,5,step_reward]
            self.add(experience)
            old_tuple = tuple

        is_end = False
        while not is_end:

            #random assignment if shoot or dribble
            is_shoot = self.is_chance(0.5)
            num=0
            if is_shoot:
                self.sequ = self.sequ + "s"
                # shoot the ball
                print("shoot")
                shoot_speed=70
                dist_ball, dist_goal,is_shoot = self.get_tuple_go_to_ball_shoot(dist_ball, dist_goal+0.5, 0, num_go_to_first_ball,shoot_speed)
                tuple = [1, 1, 1, dist_ball, dist_goal]
                experience = [old_tuple, tuple, 3, step_reward]
                self.add(experience)
                old_tuple = tuple

                num_go_to_ball_shoot = 70
                for i in range(0, int(num_go_to_ball_shoot)): #gototheball ater the shot
                    dist_ball, dist_goal,shoot_speed = self.get_tuple_go_to_ball_shoot(dist_ball, dist_goal, i+1, num_go_to_first_ball,shoot_speed)
                    tuple = [1, 1, 1, dist_ball, dist_goal]
                    experience = [old_tuple, tuple, 5, step_reward]
                    self.add(experience)
                    old_tuple = tuple
                    is_end = dist_goal - dist_ball <= 0.0
                    if is_end:
                        return
            else:
                self.sequ = self.sequ+"d"
                print("dribble")
                # dribble the ball
                dribble_speed = 40
                dist_ball, dist_goal,dribble_speed = self.get_tuple_go_to_ball_dribble(dist_ball, dist_goal+0.5, 0, num_go_to_first_ball,dribble_speed)
                tuple = [1, 1, 1, dist_ball, dist_goal]
                experience = [old_tuple, tuple, 2, step_reward]
                self.add(experience)
                old_tuple = tuple

                num_go_to_ball_dribble = 40
                for i in range(0, num_go_to_ball_dribble):  # gototheball ater the shot
                    dist_ball, dist_goal,dribble_speed = self.get_tuple_go_to_ball_dribble(dist_ball, dist_goal, i+1, num_go_to_first_ball,dribble_speed)
                    tuple = [1, 1, 1, dist_ball, dist_goal]
                    experience = [old_tuple, tuple, 5, step_reward]
                    self.add(experience)
                    old_tuple = tuple
                    is_end  = dist_goal - dist_ball <= 0.0
                    if is_end:
                        return