import numpy


class ExplorationStrategies(object):
    def __init__(self):
        self.pre_train = 32
        self.startE = 1
        self.endE = 0.0
        self.anneling_steps = 250000
        self.epsilon = self.startE
        self.stepDrop = (self.startE - self.endE) / self.anneling_steps

    def e_greedy_pre_train(self,counter, num_actions):
        random_value = numpy.random.rand(1)
        changed = False
        if (random_value < self.epsilon or counter < self.pre_train) and num_actions > 0:
            best_action = numpy.random.randint(num_actions)
            changed = True
        if self.epsilon > self.endE and self._step_counter > self.pre_train and num_actions > 0:
            self.epsilon -= self.stepDrop

        return changed, best_action

    def e_greedy(self, counter, num_actions):
        # random selection for exploration. e-greedy - strategy
        epsilon = 1. / ((counter / 50.0) + 10)
        changed = False
        random_value = numpy.random.rand(1)

        # if randomly chosen give one random action the max_activation
        if random_value < epsilon and num_actions > 0:
            best_action = numpy.random.randint(num_actions)
            changed = True
        return changed, best_action