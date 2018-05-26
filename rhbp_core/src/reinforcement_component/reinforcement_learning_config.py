class RLConstants:
    #todo umbenenen config.
    #todo init als casse
    # todo get values from rospy param
    # path to model
    model_path=""
    model_directory=""
    # setting for the nn
    number_hidden_layer=1
    number_variables=16
    auto_number_variables=True
    # Set learning parameters
    learning_rate_optimizer = 0.1
    epsilon = 0.1
    learning_rate_q_learning = 0.99
    interval_prints = 100
    microbatch_size = 20


class NNConfig(): #TODO let config get variables from rospy try-catch if rospy is avialabe if node is avialable
    def __init__(self):
        self.model_path = ""
        self.model_directory = ""
        # setting for the nn
        self.number_hidden_layer = 1
        self.number_variables = 16
        self.auto_number_variables = True
        # Set learning parameters
        self.learning_rate_optimizer = 0.1
        self.epsilon = 0.1
        self.learning_rate_q_learning = 0.99
        self.interval_prints = 100
        self.y = .99  # Discount factor.
        self.tau = 0.001  # Amount to update target network at each step.
        self.batch_size = 32  # Size of training batch
        self.buffer_size = 10000  # size of the experience learning buffer

class ExplorationConfig(): #TODO let config get variables from rospy try-catch if rospy is avialabe if node is avialable
    def __init__(self):
        self.pre_train = 10 # let the model choose random actions and dont train for these number of steps
        self.startE = 1
        self.endE = 0.05
        self.anneling_steps = 50000 # steps until it reache endE
        self.stepDrop = (self.startE - self.endE) / self.anneling_steps
        self.train_interval = 5 #train the model every train_interval steps
