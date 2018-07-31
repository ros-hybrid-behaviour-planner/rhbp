#!/usr/bin/env python2
# this is the main file of a rhbp agent instance
# it contains the agents variables, topics subscriber and publisher, as well as the regarding callback function
# Although the job_execution process is start in this file
from __future__ import division  # force floating point division when using plain /
from agent_modules_tests import *
from behaviour_components.activators import BooleanActivator, StringActivator, ThresholdActivator, GreedyActivator, \
    LinearActivator, EqualActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition, MultiSensorCondition, Disjunction, Conjunction
from behaviour_components.sensors import *
from behaviour_components.goals import OfflineGoal, GoalBase
from reinforcement_component.rl_component_tests.test_suite.rhbp_agent_base import RhbpAgentBase
from rcs_ros_bridge.msg import SimStart, GenericAction, PlayMode, Goals, Flags, Lines

import rospy

from reinforcement_component.rl_component_tests.test_suite.test_environment import RewardSensor


class TaxiAgent(RhbpAgentBase):
    def __init__(self):
        super(TaxiAgent, self).__init__()
        rospy.logdebug("RhbpAgent::init")

    def init_behaviors(self):
        """
        here we could also evaluate the msg_old in order to initialize depending on the role etc.
        :param msg:  the message
        :type msg: SimStart
        """
        self.environment_name = 'Taxi-v2'
        self.init_environment(self.environment_name)

        self.state_sensor = Sensor(name="StateSensor", state_space=500)
        reward_sensor = RewardSensor(name="RewardSensor", intervall=10)
        reward_sensor.update(0)

        action_one_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionZero",
                                                 reward_sensor=reward_sensor,
                                                 action_index=0, state_sensor=self.state_sensor, environment=self.env)

        action_two_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionOne",
                                                 reward_sensor=reward_sensor,
                                                 action_index=1, state_sensor=self.state_sensor, environment=self.env)

        action_three_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionTwo",
                                                   reward_sensor=reward_sensor,
                                                   action_index=2, state_sensor=self.state_sensor, environment=self.env)

        action_four_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionThree",
                                                  reward_sensor=reward_sensor,
                                                  action_index=3, state_sensor=self.state_sensor, environment=self.env)

        action_five_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionFour",
                                                  reward_sensor=reward_sensor,
                                                  action_index=4, state_sensor=self.state_sensor, environment=self.env)
        action_six_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionFive",
                                                 reward_sensor=reward_sensor,
                                                 action_index=5, state_sensor=self.state_sensor, environment=self.env)

        is_in_goal_state = Condition(self.state_sensor,
                                     ThresholdActivator(thresholdValue=15))  # for getting the state of the game

        good_pick_off = Condition(reward_sensor,
                                  ThresholdActivator(thresholdValue=20))  # successful dropoff
        illegal_action = Condition(reward_sensor,
                                   ThresholdActivator(thresholdValue=-10, isMinimum=False))  # wrong action

        # timestep_action = Conjunction(Negation(good_pick_off),Negation(illegal_action)) #everything else
        aboveminusone = Condition(reward_sensor,
                                  ThresholdActivator(thresholdValue=-1))
        belowminusone = Condition(reward_sensor,
                                  ThresholdActivator(thresholdValue=-1, isMinimum=False))
        timestep_action = Conjunction(aboveminusone, belowminusone)  # everything else

        the_goal = GoalBase(name="goal_the", permanent=True,
                            conditions=[is_in_goal_state], priority=0,
                            plannerPrefix=self.prefix)

        the_goal2 = GoalBase(name="dropoff_goal", permanent=True,
                             conditions=[good_pick_off], priority=20,
                             plannerPrefix=self.prefix)
        the_goal3 = GoalBase(name="illegal_goal", permanent=True,
                             conditions=[illegal_action], priority=-10,
                             plannerPrefix=self.prefix)
        the_goal4 = GoalBase(name="timestep_goal", permanent=True,
                             conditions=[timestep_action], priority=-1,
                             plannerPrefix=self.prefix)

        direct_to_goal_effect = Effect(sensor_name=is_in_goal_state.getFunctionNames()[0], indicator=1.0,
                                       sensor_type=float)  #

        action_two_behavior.correlations.append(direct_to_goal_effect)
        action_one_behavior.correlations.append(direct_to_goal_effect)
        action_three_behavior.correlations.append(direct_to_goal_effect)
        action_four_behavior.correlations.append(direct_to_goal_effect)

        direct_to_goal_effect2 = Effect(sensor_name=good_pick_off.getFunctionNames()[0], indicator=1.0,
                                        sensor_type=float)  #

        action_two_behavior.correlations.append(direct_to_goal_effect2)
        action_one_behavior.correlations.append(direct_to_goal_effect2)
        action_three_behavior.correlations.append(direct_to_goal_effect2)
        action_four_behavior.correlations.append(direct_to_goal_effect2)

        direct_to_goal_effect3 = Effect(sensor_name=illegal_action.getFunctionNames()[0], indicator=-1.0,
                                        sensor_type=float)  #

        action_two_behavior.correlations.append(direct_to_goal_effect3)
        action_one_behavior.correlations.append(direct_to_goal_effect3)
        action_three_behavior.correlations.append(direct_to_goal_effect3)
        action_four_behavior.correlations.append(direct_to_goal_effect3)

        direct_to_goal_effect4 = Effect(sensor_name=aboveminusone.getFunctionNames()[0], indicator=1.0,
                                        sensor_type=float)  #

        action_two_behavior.correlations.append(direct_to_goal_effect4)
        action_one_behavior.correlations.append(direct_to_goal_effect4)
        action_three_behavior.correlations.append(direct_to_goal_effect4)
        action_four_behavior.correlations.append(direct_to_goal_effect4)
        action_six_behavior.correlations.append(direct_to_goal_effect4)
        action_five_behavior.correlations.append(direct_to_goal_effect4)

        direct_to_goal_effect5 = Effect(sensor_name=belowminusone.getFunctionNames()[0], indicator=-1.0,
                                        sensor_type=float)  #

        action_two_behavior.correlations.append(direct_to_goal_effect5)
        action_one_behavior.correlations.append(direct_to_goal_effect5)
        action_three_behavior.correlations.append(direct_to_goal_effect5)
        action_four_behavior.correlations.append(direct_to_goal_effect5)
        action_six_behavior.correlations.append(direct_to_goal_effect5)
        action_five_behavior.correlations.append(direct_to_goal_effect5)
