import tensorflow as tf
import numpy as np
class InputStateTransformer:

    def __init__(self,manager):
        self.manager = manager
        print("init transformer")
        self.conditions={}
    def get_current_state(self):
        # TODO implement connection to sensor and conditions
        # TODO Get Preconditions of behaviors from the BehaviorBase. Needs to be sent.
        # TODO check then also for double preconditions
        # TODO question: only conditions influencing this network behavior or from all
        for behaviour in self.manager.behaviours:
            behaviour.fetchState(0) # 0 is just dummy. Note: delete that a number is needed
            for cond in behaviour.condition_values:
                self.conditions[cond.name]=cond.activation

        state = np.zeros(len(self.conditions))
        index=0
        for k,v in self.conditions.iteritems():
            print(k,v)
            state[index]=v
            index += 1

        print(state,state.shape)
        state = state.reshape([1,len(self.conditions)])
        print(state,state.shape)
        print(np.identity(16)[0:0+1],np.identity(16)[0:0+1].shape)
        test=np.identity(4)[0:0 + 1]
        return state

    def get_last_action(self):
        return 0

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