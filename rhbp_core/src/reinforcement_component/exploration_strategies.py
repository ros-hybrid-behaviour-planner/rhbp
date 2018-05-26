import numpy

from reinforcement_component.reinforcement_learning_config import ExplorationConfig


class ExplorationStrategies(object):
    def __init__(self):
        self.config = ExplorationConfig()
        self.epsilon = self.config.startE

    def e_greedy_pre_train(self,counter, num_actions):
        random_value = numpy.random.rand(1)
        best_action = None
        changed = False

        if (random_value < self.epsilon or counter < self.config.pre_train) and num_actions > 0:
            best_action = numpy.random.randint(num_actions)
            changed = True
        if self.epsilon > self.config.endE and counter > self.config.pre_train and num_actions > 0:
            self.epsilon -= self.config.stepDrop
        print("epsilon", self.epsilon,counter)
        return changed, best_action

    def e_greedy(self, counter, num_actions):
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

    def random(self,counter,num_actions):
        changed = True
        best_action = numpy.random.randint(num_actions)
        return changed,best_action

    def boltzmann(self,activations,num_actions):
        # TODO needs access to the neural network . does not have in currrent architecture
        # Choose an action probabilistically, with weights relative to the Q-values.
        #Q_d, allQ = sess.run([q_net.Q_dist, q_net.Q_out],
        #                     feed_dict={q_net.inputs: [s], q_net.Temp: e, q_net.keep_per: 1.0})
        #a = numpy.random.choice(Q_d[0], p=Q_d[0])
        #a = numpy.argmax(Q_d[0] == a)
        return