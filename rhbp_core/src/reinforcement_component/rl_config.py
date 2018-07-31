import rospy


class NNConfig(object):
    """
    used for setting the neural network 
    tries to get the values from the rospy param. if no connect can be made itt used defualt values
    can be extended to configure the layers of the neural network or the optimizers
    """

    def __init__(self):
        try:
            self.learning_rate_optimizer = rospy.get_param("~learning_rate_optimizer",
                                                           0.0001)  # learning rate of the optimizer
        except Exception:
            self.learning_rate_optimizer = 0.0001


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
            self.buffer_size = rospy.get_param("~buffer_size", 50000)  # size of the experience learning buffer
            self.train_interval = rospy.get_param("~train_interval", 5)  # train the model every train_interval steps
            self.stop_training = rospy.get_param("~stop_training",
                                                 6000000)  # steps after the model does not get trained anymore
            self.pre_train = rospy.get_param("~pre_train", 1000)  # no training before this many steps
        except Exception:
            self.y = 0.99  # Discount factor.
            self.tau = 0.001  # Amount to update target network at each step.
            self.batch_size = 32  # Size of training batch
            self.buffer_size = 5000  # size of the experience learning buffer
            self.train_interval = 5  # train the model every train_interval steps
            self.stop_training = 6000000  # steps after the model does not get trained anymore
            self.pre_train = 1000  # no training before this many steps


class SavingConfig(object):
    """
    used for saving the model 
    """

    def __init__(self):
        try:
            self.model_path = rospy.get_param("~model_path", 'models/rl-model')  # path of the saved model
            self.model_directory = rospy.get_param("~model_directory", './models')  # directory of the saved model
            self.save = rospy.get_param("~save", False)  # if the model should be saved
            self.save_buffer = rospy.get_param("~save_buffer", False)  # if the buffer should be saved
            self.steps_save = rospy.get_param("~steps_save", 100000)  # interval for saving model
            self.load = rospy.get_param("~load", False)  # if the model should be loaded
        except Exception:
            self.model_path = 'models/rl-model'  # path of the saved model
            self.model_directory = './models'  # directory of the saved model
            self.save = False  # if the model should be saved
            self.save_buffer = False  # if the buffer should be saved
            self.steps_save = 100000  # interval for saving model
            self.load = False  # if the model should be loaded


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
            self.plot_rewards = rospy.get_param("~plot_rewards", True)  # if rewards should be plotted
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
            self.pre_train = rospy.get_param("~pre_train", 1000)
            self.startE = rospy.get_param("~startE", 1.00)
            self.endE = rospy.get_param("~endE", 0.01)
            self.anneling_steps = rospy.get_param("~anneling_steps", 300000)  # steps until it reache endE
            # function that describes the stepDrop changing epsilon
            self.stepDrop = (self.startE - self.endE) / self.anneling_steps
        except Exception:
            self.pre_train = 1000
            self.startE = 1.00
            self.endE = 0.0
            self.anneling_steps = 300000  # steps until it reache endE
            # function that describes the stepDrop changing epsilon
            self.stepDrop = (self.startE - self.endE) / self.anneling_steps


class TransitionConfig(object):
    """
    used to set parameter which set the transitioning from rhbp to the rl component
    """

    def __init__(self):
        try:
            self.use_wishes = rospy.get_param("~use_wishes", False)
            self.use_true_values = rospy.get_param("~use_true_values", True)
            self.max_activation = rospy.get_param("~max_activation", 1)
            self.min_activation = rospy.get_param("~min_activation", -1)
            self.weight_rl = rospy.get_param("~weight_rl", 1.0)
        except Exception:
            self.use_wishes = False
            self.use_true_values = True
            self.max_activation = 1
            self.min_activation = -1
            self.weight_rl = 1.0
