class RLConstants:
    def __init__(self):
        # path to model
        self.model_path=""
        self.model_directory=""
        # setting for the nn
        self.number_hidden_layer=1
        self.number_variables=16
        self.auto_number_variables=True
        # Set learning parameters
        self.learning_rate_optimizer = 0.1
        self.epsilon = 0.1
        self.learning_rate_q_learning = 0.99