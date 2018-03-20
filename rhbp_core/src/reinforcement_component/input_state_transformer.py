import tensorflow as tf
import numpy as np
class InputStateTransformer:

    def __init__(self,manager):
        self.manager = manager
        print("init transformer")

    def get_current_state(self):
        # TODO implement connection to sensor and conditions
        # TODO Get Preconditions of behaviors from the BehaviorBase. Needs to be sended.
        # TODO check then also for double preconditions
        # TODO question: only conditions influencing this network behavior or from all
        example_state = np.identity(16)[0:1]

        return example_state

    def get_reward_from_state(self):
        """
        this function calculates regarding the fulfillment and priorities of the active goals
        a reward value. it prioritizes the goals in  a way that a higher priority is always more important
        than a smaller one (power of 10)
        :return: 
        """
        # TODO think about better logic maybe

        reward_value = 0
        for goal in self.manager.get_goals():
            goal_value = goal.fulfillment * (10 ** goal.priority)
            reward_value += goal_value
        return reward_value

    def get_random_action(self):

        return np.random.randint(len(self.manager.get_behaviors())+1)