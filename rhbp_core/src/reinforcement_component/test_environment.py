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

from reinforcement_component.input_state_transformer import SensorValueTransformer
from reinforcement_component.nn_model_base import ReinforcementAlgorithmBase
from reinforcement_component.reinforcement_learning_config import ExplorationConfig, NNConfig

# class for revisiting experiences
class environment_test():
    def __init__(self, buffer_size=10000):
        self.buffer = []
        self.buffer_size = buffer_size
        self.counter = 0
        self.print_steps = False
        random.seed(0)
        self.sim_counter = 0
        self.dict_sequ = {"ds":(0,0),"ss":(0,0),"ds":(0,0),"ddd":(0,0),"dds":(0,0),"sd":(0,0)}
        self.sequ = ""
        self.is_zero = False

    def get_new_tuple(self,t,a):
        d = t[0]
        if a == 3:
            r = -1.5
        else:
            r = -0.2
        if a == 2:
            d -= 10

        else:
            d -=35

        if d <=0:
            d = 60
            r = 1
        #print(t,a,d,r)
        if d == 60 and not self.is_zero:
            self.is_zero = True
            return [[d, 0], t, a, r]
        if t[0]==60 and t[1]==0:
            self.is_zero=False
            return [[60, 1], t, a, 0.1]
        return [ [d,1],t,a, r]




    def add_easy_trial(self):
        neg_value = -1
        no_goal_value = -0.1
        for i in range(0, 60):
            new_i = i - 10
            if new_i < 0:
                new_i = 60

            if not i == 60 and new_i == 60:
                r = 1
            else:
                r = no_goal_value
            d = [[i, 1], [new_i, 1], 2, r]
            d2 = [[i, 0], [i, 0], 2, neg_value]
            if i < 35:
                r = 1
            else:
                r = no_goal_value
            new_i = i - 35
            # r = (i * (-1) + 35)/60.0
            if new_i < 0:
                new_i = 60
            if not i == 60 and new_i == 60:
                r = 1
            else:
                r = no_goal_value
            s = [[i, 1], [new_i, 1], 3, r]
            s2 = [[i, 0], [i, 0], 3, neg_value]
            self.add_easy_neg(i, neg_value)
            # print(d)
            # print(s)
            self.add(d)
            self.add(s)

            self.add(d2)
            self.add(s2)

    def add_easy_neg(self, i, neg_value):
        # neg_value= -100
        t = [[i, 1], [i, 1], 0, neg_value]
        self.add(t)
        t = [[i, 1], [i, 1], 1, neg_value]
        self.add(t)
        t = [[i, 1], [i, 1], 4, neg_value]
        self.add(t)
        t = [[i, 1], [i, 1], 5, neg_value]
        self.add(t)

        t = [[i, 0], [i, 0], 0, 1]
        self.add(t)
        t = [[i, 0], [i, 0], 1, 1]
        self.add(t)
        t = [[i, 0], [i, 0], 4, 1]
        self.add(t)
        t = [[i, 0], [i, 0], 5, 1]
        self.add(t)

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

    def reward_from_dist(self,num):
        return 4+num*1.5

    def add_expected_tuple(self,t,action):
        #action = numpy.random.randint(low=0,high=5)
        print(t)
        play = 1
        ball = 1
        goal = 1
        kick = 1
        dist = t[0][4]
        if action == 0: #turn_to_ball
            play=0
            kick=0

            is_state = [play,ball,goal,kick,dist]
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
            self.add_easy_trial()
            continue
            #self.add_trial()
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
        return
        negative_reward = -1
        for i in range(0,6):
            if i == experience[2] or (experience[2] ==3 and i == 2) or (experience[2] == 2 and i == 3):
                continue
            else:
                self.add( [experience[0],experience[0],i,negative_reward],True)

    def add_trial(self):
        step_reward = 0
        dist_ball = 10
        dist_goal = 60
        not_play_t = [0,1,1,dist_ball,dist_goal,0]
        reward_not = 1
        experience_stay_not_play = [not_play_t, not_play_t, 0, reward_not]
        num_wait_before_play = 2
        for i in range(0,num_wait_before_play-1):
            self.add(experience_stay_not_play)

        after_not_play_t = [1,1,1,dist_ball,dist_goal,0]
        experience_after_not_play = [not_play_t,after_not_play_t,0,-1]
        self.add(experience_after_not_play)
        old_tuple = after_not_play_t
        num_go_to_first_ball = 20.0
        for i in range(0,int(num_go_to_first_ball-1)):
            dist_ball,dist_goal = self.get_tuple_go_to_ball_start(dist_ball,dist_goal,i,num_go_to_first_ball)
            tuple = [1,1,1,dist_ball,dist_goal,0]
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
                shoot_speed=70
                dist_ball, dist_goal,is_shoot = self.get_tuple_go_to_ball_shoot(dist_ball, dist_goal+0.5, 0, num_go_to_first_ball,shoot_speed)
                tuple = [1, 1, 1, dist_ball, dist_goal,shoot_speed]
                experience = [old_tuple, tuple, 3, step_reward]
                self.add(experience)
                old_tuple = tuple

                num_go_to_ball_shoot = 70
                for i in range(0, int(num_go_to_ball_shoot)): #gototheball ater the shot
                    dist_ball, dist_goal,shoot_speed = self.get_tuple_go_to_ball_shoot(dist_ball, dist_goal, i+1, num_go_to_first_ball,shoot_speed)
                    tuple = [1, 1, 1, dist_ball, dist_goal,shoot_speed]
                    experience = [old_tuple, tuple, 5, step_reward]
                    is_end = dist_goal - dist_ball <= 0.0
                    if is_end:
                        experience[3]=1
                        self.add(experience)
                        return
                    self.add(experience)
                    old_tuple = tuple
            else:
                self.sequ = self.sequ+"d"
                # dribble the ball
                dribble_speed = 40
                dist_ball, dist_goal,dribble_speed = self.get_tuple_go_to_ball_dribble(dist_ball, dist_goal+0.5, 0, num_go_to_first_ball,dribble_speed)
                tuple = [1, 1, 1, dist_ball, dist_goal,dribble_speed]
                experience = [old_tuple, tuple, 2, step_reward]
                self.add(experience)
                old_tuple = tuple

                num_go_to_ball_dribble = 40
                for i in range(0, num_go_to_ball_dribble):  # gototheball ater the shot
                    dist_ball, dist_goal,dribble_speed = self.get_tuple_go_to_ball_dribble(dist_ball, dist_goal, i+1, num_go_to_first_ball,dribble_speed)
                    tuple = [1, 1, 1, dist_ball, dist_goal,dribble_speed]
                    experience = [old_tuple, tuple, 5, step_reward]
                    is_end = dist_goal - dist_ball <= 0.0
                    if is_end:
                        experience[3]=1
                        self.add(experience)
                        return
                    self.add(experience)
                    old_tuple = tuple
