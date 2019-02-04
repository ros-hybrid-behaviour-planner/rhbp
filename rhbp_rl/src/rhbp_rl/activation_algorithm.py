'''
Created on 22.03.2017

@author: hrabia, lehmann
'''
from __future__ import division  # force floating point division when using plain /

import rospy

import subprocess
import numpy as np
from exploration_strategies import ExplorationStrategies
from input_state_transformer import InputStateTransformer
from rl_config import TransitionConfig

from rhbp_core.msg import InputState
from rhbp_core.srv import GetActivation
from rl_component import RLComponent
from behaviour_components.activation_algorithm import BaseActivationAlgorithm, ActivationAlgorithmFactory

import utils.rhbp_logging

rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.rl')


class ReinforcementLearningActivationAlgorithm(BaseActivationAlgorithm):
    """
    This activation algorithm changes the activation calculation formulas in respect to the base algorithm with
    including the reinforcement learning 
    """

    def __init__(self, manager, extensive_logging=False, create_log_files=False):
        super(ReinforcementLearningActivationAlgorithm, self).__init__(manager, extensive_logging=extensive_logging)

        # saves list with the activations
        self.activation_rl = []
        # timeout until trying to reach the service gets stopped
        self.SERVICE_TIMEOUT = 5
        self.config = TransitionConfig()
        # values how much the rl activation influences the other components
        self.weight_rl = self.config.weight_rl
        # set if the exploration choose random action
        self.max_activation = self.config.max_activation
        # gets set for not executable actions
        self.min_activation = self.config.min_activation
        # step counter is used for exploration
        self._step_counter = 0
        # setting the activation decay.
        self._activation_decay = self.config.activation_decay
        # transforms rhbp values to rl values
        self.input_transformer = InputStateTransformer(manager)
        # the rl component
        self.rl_component = None
        # adrress of the rl node
        self.rl_address = self._manager._prefix.replace("/Manager", "_rl_node")
        # start only rl_component if needed
        if self.weight_rl > 0.0:
            # select if a own node should be started for the rl_component
            if self.config.use_node:
                rhbplog.loginfo("Starting external RL node.")
                self.start_rl_node()
                rhbplog.loginfo("External RL node started.")
            else:
                self.start_rl_class()
                rhbplog.loginfo("Using RL component directly.")
        # implements different exploration strategies
        self.exploration_strategies = ExplorationStrategies()

    def start_rl_class(self):
        """
        starts the rl_component as a class
        :return: 
        """
        rhbplog.loginfo("starting rl_component as a class")
        self.rl_component = RLComponent(name=self.rl_address)

    def start_rl_node(self):
        """
        start the rl_component as a node
        """
        rospy.loginfo("starting rl_component as a node")
        package = 'rhbp_rl'
        executable = 'src/rhbp_rl/rl_component.py'
        command = "rosrun {0} {1} _name:={2}".format(package, executable, self.rl_address)
        p = subprocess.Popen(command, shell=True)

        state = p.poll()
        if state is None:
            rhbplog.loginfo("RL process is running fine")
        elif state < 0:
            rhbplog.loginfo("RL Process terminated with error")
        elif state > 0:
            rhbplog.loginfo("RL Process terminated without error")

    def check_if_input_state_correct(self):
        """
        receive via the input state transformer the rl values needed for the algorithm.
        checks if values are incomplete
        :return: returns the input state values or a False value if vlaues are incomplete
        """
        # receive outputs from the known behaviours
        num_outputs = len(self._manager.behaviours)
        if num_outputs == 0:
            return False, None, None, None, None, None
        # receive the input values from the sensors and conditions
        input_state = self.input_transformer.transform_input_values()
        num_inputs = input_state.shape[0]
        if num_inputs == 0:
            return False, None, None, None, None, None
        # receive the reward from the goals
        reward = self.input_transformer.calculate_reward()
        # receive the executed action from the executed behaviour
        last_action = self._manager.executedBehaviours
        if len(last_action) == 0:
            return False, None, None, None, None, None
        last_action_index = self.input_transformer.behaviour_to_index(
            last_action[0])  # XXX can be expanded here to multiple executed actions
        if last_action_index is None:
            return False, None, None, None, None, None

        return True, num_inputs, num_outputs, input_state, reward, last_action_index

    def get_activation_from_rl_node(self):
        """
        this function gets first the InputState from the rhbp components and sends this to the rl_node via service.
        
        :return: the received activations
        """
        # get the input state. also check if the input state is correct
        is_correct, num_inputs, num_outputs, input_state, reward, last_action_index = self.check_if_input_state_correct()
        # if input state is incorrect return
        if not is_correct:
            return
        # create an input state message for sending it to the rl_node
        input_state_msg = InputState()
        input_state_msg.input_state = input_state
        input_state_msg.num_outputs = num_outputs
        input_state_msg.num_inputs = num_inputs
        input_state_msg.reward = reward
        input_state_msg.last_action = last_action_index

        # collect negative states (not executable behaviors)
        num_actions = len(self._manager._behaviours)
        negative_states = []
        if self.config.use_negative_states:
            for action_index in range(num_actions):
                # if random chosen number is not executable, give minus reward and sent it to rl component
                if not self._manager._behaviours[action_index].executable:
                    negative_state = InputState()
                    negative_state.input_state = input_state
                    negative_state.num_outputs = num_outputs
                    negative_state.num_inputs = num_inputs
                    negative_state.reward = self.min_activation
                    negative_state.last_action = action_index
                    negative_states.append(negative_state)

        # start service to get the activations from the model
        self.fetch_activation(input_state_msg, negative_states)

    def fetch_activation(self, input_state, negative_states):
        """
        This method fetches the activation from the RL component either directly or through
        rl node via GetActivation service call
        :param input_state:
        :param negative_states: 
        :return: 
        """
        if self.config.use_node:  # either use the service or a direct method call.
            try:
                rhbplog.logdebug("Waiting for service %s", self.rl_address + 'GetActivation')
                rospy.wait_for_service(self.rl_address + 'GetActivation', timeout=self.SERVICE_TIMEOUT)
            except rospy.ROSException:
                rospy.logerr("GetActivation service not found")
                return 0
            try:
                get_activation_request = rospy.ServiceProxy(self.rl_address + 'GetActivation', GetActivation)
                activation_state = get_activation_request(input_state, negative_states).activation_state

            except rospy.ServiceException as e:
                rhbplog.logerr("ROS service exception in 'fetchActivation' of behaviour '%s':", self.rl_address)
        else:
            activation_state = self.rl_component.get_activation_state(input_state, negative_states)

        self.activation_rl = list(activation_state.activations)

    def get_rl_activation_for_ref(self, ref_behaviour):
        index = self.input_transformer.behaviour_to_index(ref_behaviour)
        # if the activation is not yet received the values are zero
        if len(self.activation_rl) == 0:
            value = 0
        else:
            value = self.activation_rl[index]
            # normalisation
            max = np.max(self.activation_rl)
            min = np.min(self.activation_rl)
            b = self.config.max_activation
            a = self.config.min_activation
            value = (b - a) * (value - min) / (max - min) + a
            value *= self.config.weight_rl
        return value

    def compute_behaviour_activation_step(self, ref_behaviour):
        """
        computes out of the different activation influences the activation step. 
        :param ref_behaviour: for which behaviour the activation should be computed
        :return: 
        """

        activation_precondition = self.get_activation_from_preconditions(ref_behaviour)
        activation_goals = self.get_activation_from_goals(ref_behaviour)[0]
        inhibition_goals = self.get_inhibition_from_goals(ref_behaviour)[0]
        activation_predecessors = self.get_activation_from_predecessors(ref_behaviour)[0]
        activation_successors = self.get_activation_from_successors(ref_behaviour)[0]
        inhibition_conflictors = self.get_inhibition_from_conflictors(ref_behaviour)[0]
        activation_plan = self.get_activation_from_plan(ref_behaviour)[0]
        rl_activation = self.get_rl_activation_for_ref(ref_behaviour)

        rhbplog.loginfo("\t%s: activation from preconditions: %s", ref_behaviour, activation_precondition)
        rhbplog.loginfo("\t%s: activation from goals: %s", ref_behaviour, activation_goals)
        rhbplog.loginfo("\t%s: inhibition from goals: %s", ref_behaviour, inhibition_goals)
        rhbplog.loginfo("\t%s: activation from predecessors: %s", ref_behaviour, activation_predecessors)
        rhbplog.loginfo("\t%s: activation from successors: %s", ref_behaviour, activation_successors)
        rhbplog.loginfo("\t%s: inhibition from conflicted: %s", ref_behaviour, inhibition_conflictors)
        rhbplog.loginfo("\t%s: activation from plan: %s", ref_behaviour, activation_plan)
        rhbplog.loginfo("\t%s: activation from rl: %s", ref_behaviour, rl_activation)
        current_activation_step = activation_precondition \
                                  + activation_goals \
                                  + inhibition_goals \
                                  + activation_predecessors \
                                  + activation_successors \
                                  + inhibition_conflictors \
                                  + activation_plan \
                                  + rl_activation
        ref_behaviour.current_activation_step = current_activation_step
        return current_activation_step

    def update_config(self, **kwargs):
        """
        overrides update_config. includes fetching the most recent activation for the input state and 
        choosing a random action according to the exploration strategy 
        :param kwargs: 
        :return: 
        """
        # include BaseActivation functions
        super(ReinforcementLearningActivationAlgorithm, self).update_config(**kwargs)
        self._activation_decay = self.config.activation_decay
        num_actions = len(self._manager._behaviours)  # TODO this is dirty, accessing private member
        # if the rl activation is not used dont calculate the values and set all to zero
        if self.weight_rl <= 0: # TODO why this weight comes from a different source?
            self.activation_rl = [0] * num_actions  # set all activations to 0
            return

        # get the activations from the rl_component via service
        self.get_activation_from_rl_node()
        # return if no activations received
        if len(self.activation_rl) == 0:
            return

        # TODO Idea: Why not test first for exploration? Test to move it before get_activation_from_rl_node.Maybe error
        #  check " len(self.activation_rl) == 0"could make problems. Getting activation is also saving the current state
        # hence it has to be called but maybe we can avoid the feed forward through a flag in the service?!

        # check if exploration chooses randomly best action
        changed, best_action = self.exploration_strategies.e_greedy_pre_train(self._step_counter, num_actions)
        if changed:
            self.activation_rl = [0] * num_actions  # set all activations to 0
            self.activation_rl[best_action] = self.max_activation  # only support the one to be explored
        # set the step counter higher
        self._step_counter += 1


def register_in_factory(activation_algo_factory):
    """
    After this module has been imported you have to register the algos from this module in the other factory.
    :param activation_algo_factory: the factory to register the activation algorithm
    :type activation_algo_factory: ActivationAlgorithmFactory
    :return:
    """

    activation_algo_factory.register_algorithm("reinforcement", ReinforcementLearningActivationAlgorithm)
