from nn_model import ModelNeuralNetwork
from input_state_transformer import InputStateTransformer
import time
class RLComponent:

    def __init__(self, manager,name="rl_component"):

        print("started rl component",name)
        self.manager = manager
        self.model = None
        self.behaviors = None
        self.name = name
        self.is_model_init = False
        time.sleep(2)
    def start_learning(self):
        time.sleep(1)
        self.model = ModelNeuralNetwork(self.name)
        self.model.transformer = InputStateTransformer(self.manager)

        if not self.is_model_init:
            self.init_model()

        # TODO get current state and feed forward and then get next state and updat emodel

    def init_model(self):

        self.behaviors = self.manager.get_behaviors()
        self.goals = self.manager.get_goals()


        # TODO get dimensions from manager
        #for b in self.behaviors:
        #    print(b)


        start_input = self.model.transformer.get_current_state()
        self.number_inputs = start_input.shape[1]

        self.number_behaviors = len(self.manager._behaviours)
        if self.number_behaviors <= 1 or self.number_inputs <= 1:
            print("not enough behaviors or inputs")
            return

        print("num behaviors", self.number_behaviors, "num inputs", self.number_inputs, "num goals", len(self.goals))
        for g in self.goals:
            print(g.name, g.activated, g.priority, g.fulfillment, g.active)

        self.model.behaviors = self.behaviors

        model_exists = self.model.check_if_model_exists()
        # return
        if model_exists:
            self.model.load_model()
        else:
            self.model.initialize_model()
        self.is_model_init = True

    def get_model_parameters(self):
        #self.model.transformer.get_current_state()
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
        if self.is_model_init:
            # update outputs of model. see if something changed
            self.behaviors=self.manager.behaviours

            # update active behaviors. these are the ones that got executed in the last step
            self.model.active_behaviors = map(self.behaviour_to_index,self.manager.executedBehaviours)
            print("-----------------------")
            print("1",self.behaviors)
            print("2",self.manager.executedBehaviours)
            print("3",self.model.active_behaviors)
            print("-----------------------")

            self.model.behaviors = self.manager.behaviours
            # train the model
            self.model.train_model()
            # get activation for current state
            self.activations = self.model.feed_forward()
            # return only activation for asked ref_behavior
            activation=self.activations[self.behaviour_to_index(ref_behaviour.name)]
            return activation
        else:
            self.init_model()