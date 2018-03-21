from nn_model import ModelNeuralNetwork
from input_state_transformer import InputStateTransformer

class RLComponent:

    def __init__(self, manager,name="rl_component"):

        print("started rl component",name)
        self.manager = manager
        self.model = None
        self.behaviors = None
    def start_learning(self):

        self.model = ModelNeuralNetwork()
        self.behaviors = self.manager.get_behaviors()
        self.goals = self.manager.get_goals()


        # TODO get dimensions from manager
        for b in self.behaviors:
            print(b)

        number_inputs = 16 #TODO find correct one
        number_behaviors = len(self.manager._behaviours)
        self.model.transformer = InputStateTransformer(self.manager)
        print("num behaviors",number_behaviors,"num inputs",number_inputs,"num goals",len(self.goals))
        for g in self.goals:
            print(g.name,g.activated,g.priority,g.fulfillment,g.active)


        self.model.update_dimensions(number_inputs,number_behaviors)
        self.model.transformer = InputStateTransformer(self.manager)
        self.model.transformer.get_current_state()
        model_exists = self.model.check_if_model_exists()
        return
        if model_exists:
            self.model.load_model()
        else:
            self.model.initialize_model()


        # TODO get current state and feed forward and then get next state and updat emodel
    def get_model_parameters(self):
        self.model.transformer.get_current_state()
        return
        self.behaviors = self.manager.get_behaviors()
        self.goals = self.manager.get_goals()

        # TODO get dimensions from manager
        number_inputs = 16  # TODO find correct one. use transformer class for that?
        number_behaviors = len(self.manager._behaviours)

        for g in self.goals:
            print(g.name,g.activated,g.priority,g.fulfillment,g.active)
        print(self.model.transformer.get_reward_from_state())
        return
        print("num behaviors", number_behaviors, "num inputs", number_inputs, "num goals", len(self.goals))

        for b in self.behaviors:
            print(b,self.behaviour_to_index(b),b.name,b.activation,b.isExecuting,b._activationFromPreconditions)

    def behaviour_to_index(self,name):
        num = 0
        for b in self.behaviors:
            if b == name:
                return num
            num += 1
        return None

    def get_rl_activation(self,ref_behaviour):
        """
        load regarding the current state the activations.
        
        :return: 
        """
        # TODO  check if behaviors or goals have changed . e.g. by length. if yes reinitialize

        self.activations = self.model.feed_forward()
