"""
The different configuration parameters. They get Whether here set or as ros parameters
@author: lehmann
"""
import rospy


class NNConfig(object):
    """
    used for setting the neural network 
    tries to get the values from the rospy param. if no connect can be made itt used defualt values
    can be extended to configure the layers of the neural network or the optimizers
    """

    def __init__(self):
        try:
            # True if AdamOptimizer should be used. Uses GradientDescentOptimizer otherwise
            self.use_adam_optimizer = rospy.get_param("~use_adam_optimizer", True)
            self.learning_rate_optimizer = rospy.get_param("~learning_rate_optimizer",
                                                           0.001)  # learning rate of the optimizer

        except Exception:  # catches if no RosService was found
            self.learning_rate_optimizer = 0.001
            self.use_adam_optimizer = True


class DQNConfig(object):
    """
    sets parameter for configuring DQN
    """

    def __init__(self):
        try:
            # Set learning parameters
            self.y = rospy.get_param("~y", 0.99)  # Discount factor.
            self.tau = rospy.get_param("~tau", 0.001)  # Amount to update target network at each step.
            self.batch_size = rospy.get_param("~batch_size", 32)  # Size of training batch
            self.buffer_size = rospy.get_param("~buffer_size", 10000)  # size of the experience learning buffer
            self.train_interval = rospy.get_param("~train_interval", 5)  # train the model every train_interval steps
            self.stop_training = rospy.get_param("~stop_training",
                                                 6000000)  # steps after the model does not get trained anymore
            self.pre_train = rospy.get_param("~pre_train", 100)  # no training before this many steps
        except Exception:
            self.y = 0.99  # Discount factor.
            self.tau = 0.001  # Amount to update target network at each step.
            self.batch_size = 32  # Size of training batch
            self.buffer_size = 10000  # size of the experience learning buffer
            self.train_interval = 5  # train the model every train_interval steps
            self.stop_training = 6000000  # steps after the model does not get trained anymore
            self.pre_train = 100  # no training before this many steps


class SavingConfig(object):
    """
    used for saving the model 
    """

    def __init__(self):
        try:
            self.model_path = rospy.get_param("~model_path", 'models/rl-model')  # path of the saved model
            self.model_directory = rospy.get_param("~model_directory", './models')  # directory of the saved model
            self.save = rospy.get_param("~save", True)  # if the model should be saved
            self.save_buffer = rospy.get_param("~save_buffer", True)  # if the buffer should be saved
            self.steps_save = rospy.get_param("~steps_save", 10000)  # interval for saving model
            self.load = rospy.get_param("~load", True)  # if the model should be loaded
        except Exception:
            self.model_path = 'models/rl-model'  # path of the saved model
            self.model_directory = './models'  # directory of the saved model
            self.save = True  # if the model should be saved
            self.save_buffer = True  # if the buffer should be saved
            self.steps_save = 10000  # interval for saving model
            self.load = True  # if the model should be loaded


class EvaluationConfig(object):
    """
    used for saving loss error and reward plots
    """

    def __init__(self):
        try:
            self.eval_step_interval = rospy.get_param("~eval_step_interval",
                                                      500)  # intervall for plotting current results
            self.eval_mean_size = rospy.get_param("~eval_mean_size",
                                                  5000)  # number of plotting mean for loss and rewards

            self.plot_rewards = rospy.get_param("~plot_rewards", False)  # if rewards should be plotted
            self.plot_loss = rospy.get_param("~plot_loss", True)  # if loss should be plotted
        except Exception:
            self.eval_step_interval = 10000  # intervall for plotting current results
            self.eval_mean_size = 5000  # number of plotting mean for loss and rewards
            self.plot_rewards = True  # if rewards should be plotted
            self.plot_loss = True  # if loss should be plotted


class ExplorationConfig(object):
    """
    used for setting the parameter for the exploration strategy
    """

    def __init__(self):
        try:
            # let the model choose random actions and dont train for these number of steps
            self.pre_train = rospy.get_param("~pre_train", 000)
            self.startE = rospy.get_param("~startE", 0.50)
            self.endE = rospy.get_param("~endE", 0.01)
            self.anneling_steps = rospy.get_param("~anneling_steps", 100000)  # steps until it reache endE
            # function that describes the stepDrop changing epsilon
            self.stepDrop = (self.startE - self.endE) / self.anneling_steps
        except Exception:
            self.pre_train = 000
            self.startE = 0.00
            self.endE = 0.0
            self.anneling_steps = 10000  # steps until it reache endE
            # function that describes the stepDrop changing epsilon
            self.stepDrop = (self.startE - self.endE) / self.anneling_steps


class TransitionConfig(object):
    """
    used to set parameter which set the transitioning from rhbp to the rl component
    """

    def __init__(self):
        try:
            self.use_wishes = rospy.get_param("~use_wishes", False)  # if the transformer should use wishes as input
            self.use_true_values = rospy.get_param("~use_true_values",
                                                   True)  # if the transformer should use conditions as input
            self.max_activation = rospy.get_param("~max_activation", 1)  # maximal activation
            self.min_activation = rospy.get_param("~min_activation", 0)  # minimal activation
            self.weight_rl = rospy.get_param("~weight_rl", 1.0)  # weight of the rl component
            self.activation_decay = rospy.get_param("~activation_decay",
                                                    0.0)  # how much to way the current activation for the next activation step
            self.use_negative_states = rospy.get_param("~use_negative_states",
                                                       True)
            self.use_node = rospy.get_param("~use_node", False)  # if a own node should be used for the rl_component
        except Exception:
            self.use_wishes = False
            self.use_true_values = True
            self.max_activation = 1
            self.min_activation = 0
            self.weight_rl = 1.0
            self.activation_decay = 0.0
            self.use_negative_states = True
            self.use_node = False
