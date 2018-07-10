import rospy


class NNConfig():
    """
    used for setting the neural network and the algorithm parameter
    """

    def __init__(self):
        # path for saving the trained model
        self.model_path = rospy.get_param("~model_path", 'models/rl-model')
        self.model_directory = rospy.get_param("~model_directory", './models')
        # setting for the nn
        self.number_hidden_layer = rospy.get_param("~number_hidden_layer", 1)  # not in use
        self.number_variables_hl = rospy.get_param("~number_variables_hl", 64)  # not in use
        # Set learning parameters
        self.y = rospy.get_param("~y", 0.99)  # Discount factor.
        self.tau = rospy.get_param("~tau", 0.001)  # Amount to update target network at each step.
        self.batch_size = rospy.get_param("~batch_size", 75)  # Size of training batch
        self.buffer_size = rospy.get_param("~buffer_size", 50000)  # size of the experience learning buffer
        self.steps_save = rospy.get_param("~steps_save", 500)  # interval for saving model
        self.save = rospy.get_param("~save", False)  # if the model should be saved
        self.print_model = rospy.get_param("~print_model", True)  # if the model should be saved
        self.steps_prints = rospy.get_param("~steps_prints", 500)  # interval for saving model
        self.experiment_steps = rospy.get_param("~experiment_steps", 40000000)
        self.train_interval = rospy.get_param("~train_interval", 50)  # train the model every train_interval steps
        self.stop_training = rospy.get_param("~stop_training",
                                             6000000)  # steps after the model does not get trained anymore
        self.pre_train = rospy.get_param("~pre_train", 10000)  # no training before this many steps


class ExplorationConfig():
    """
    used for setting the parameter for the exploration strategy
    """

    def __init__(self):
        # let the model choose random actions and dont train for these number of steps
        self.pre_train = rospy.get_param("~pre_train", 10000)
        self.startE = rospy.get_param("~startE", 1.00)
        self.endE = rospy.get_param("~endE", 0.0)
        self.anneling_steps = rospy.get_param("~anneling_steps", 1500000)  # steps until it reache endE
        # function that describes the stepDrop changing epsilon
        self.stepDrop = (self.startE - self.endE) / self.anneling_steps
