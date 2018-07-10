import numpy

from reinforcement_component.reinforcement_learning_config import ExplorationConfig


class ExplorationStrategies(object):
    def __init__(self):
        self.config = ExplorationConfig()
        self.epsilon = self.config.startE

    def e_greedy_pre_train(self,counter, num_actions):
        """
        this function chooses a random action, with a decreasing epsilon and a pretrain phase.
         In the pre train phase only random actions are choosen
        :param counter: which step it is, to ideantifying how large the epsilon should be and if still in the pre_train phase
        :param num_actions: number of possible actions
        :return: if an action was selected and which
        """
        random_value = numpy.random.rand(1)
        best_action = None
        changed = False

        if (random_value < self.epsilon or counter < self.config.pre_train) and num_actions > 0:
            best_action = numpy.random.randint(num_actions)
            changed = True
        if self.epsilon > self.config.endE and counter > self.config.pre_train and num_actions > 0:
            self.epsilon -= self.config.stepDrop
        #print("epsilon", self.epsilon,counter)
        return changed, best_action

    def e_greedy(self, counter, num_actions):
        """
        this function chooses a random action, with a decreasing epsilon 
        :param counter: which step it is, to identifying how large the epsilon should be
        :param num_actions: number of possible actions
        :return: if an action was selected and which
        """
        # random selection for exploration. e-greedy - strategy
        epsilon = 1. / ((counter / 50.0) + 10)
        changed = False
        random_value = numpy.random.rand(1)
        best_action = None

        # if randomly chosen give one random action the max_activation
        if random_value < epsilon and num_actions > 0:
            best_action = numpy.random.randint(num_actions)
            changed = True
        return changed, best_action

    def random(self,num_actions):
        """
        this function just chooses random actions
        :param num_actions:  number of possible actions
        :return: 
        """
        changed = True
        best_action = numpy.random.randint(num_actions)
        return changed,best_action
