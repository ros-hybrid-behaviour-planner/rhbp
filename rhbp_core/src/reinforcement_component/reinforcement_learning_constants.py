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

