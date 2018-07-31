#!/usr/bin/env python2
# this is the main file of a rhbp agent instance
# it contains the agents variables, topics subscriber and publisher, as well as the regarding callback function
# Although the job_execution process is start in this file
from __future__ import division # force floating point division when using plain /
from behaviour_components.activators import BooleanActivator, ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition, Disjunction, Conjunction
from behaviour_components.sensors import *
from behaviour_components.goals import GoalBase
from rhbp_agent_base import RhbpAgentBase
from rcs_ros_bridge.msg import SimStart

import rospy

from reinforcement_component.rl_component_tests.test_suite.test_environment import RewardSensor, BehaviorShell, \
    TestTaxiEnv, LocationCondition


class TaxiAgentManagerAllRHBPDecoded(RhbpAgentBase):
    def __init__(self):
        super(TaxiAgentManagerAllRHBPDecoded, self).__init__()
        rospy.logdebug("RhbpAgent::init")

    def start_simulation(self):
        self.init_behaviors()
        self.env.seed(0)
        state = self.env.reset()
        self.state_sensor.update(state)
        print("init env in state", state)
        self.test_env.start_simulation()

    def get_equal_condition(self, sensor, value):
        above = Condition(sensor,
                          ThresholdActivator(thresholdValue=value, name="aboveactiv"))
        below = Condition(sensor,
                          ThresholdActivator(thresholdValue=value, isMinimum=False, name="belowactiv"))
        equal = Conjunction(above, below)  # everything else
        return equal


    def init_behaviors(self):
        """
        here we could also evaluate the msg_old in order to initialize depending on the role etc.
        :param msg:  the message
        :type msg: SimStart
        """
        self.environment_name = 'Taxi-v2'
        #self.environment_name = 'Taxi-v2'
        self.init_environment(self.environment_name)

        self.state_sensor = Sensor(name="StateSensor")
        self.state_sensor.rl_extension = RlExtension(include_in_rl=False,state_space=500,encoding=EncodingConstants.HOT_STATE)
        reward_sensor = RewardSensor(name="RewardSensor",intervall=100)
        reward_sensor.rl_extension = RlExtension(include_in_rl=False)
        reward_sensor.update(0)

        # undecoded state sensors
        self.row_state_sensor = Sensor(name="RowStateSensor")
        row_state_rl_extension = RlExtension(state_space=5, include_in_rl=True,encoding=EncodingConstants.HOT_STATE)
        self.row_state_sensor.rl_extension = row_state_rl_extension

        self.col_state_sensor = Sensor(name="ColStateSensor")
        col_state_rl_extension = RlExtension(state_space=5, include_in_rl=True,encoding=EncodingConstants.HOT_STATE)
        self.col_state_sensor.rl_extension = col_state_rl_extension

        self.passenger_state_sensor = Sensor(name="PassengerStateSensor")
        passenger_state_rl_extension = RlExtension(state_space=5, include_in_rl=True,encoding=EncodingConstants.HOT_STATE)
        self.passenger_state_sensor.rl_extension = passenger_state_rl_extension

        self.destination_state_sensor = Sensor(name="DestinationStateSensor")
        destination_state_rl_extension = RlExtension(state_space=4, include_in_rl=True,encoding=EncodingConstants.HOT_STATE)
        self.destination_state_sensor.rl_extension = destination_state_rl_extension

        # init sensor with variables for state 26
        self.row_state_sensor.update(0)
        self.col_state_sensor.update(1)
        self.passenger_state_sensor.update(1)
        self.destination_state_sensor.update(2)

        state_list = [self.row_state_sensor, self.col_state_sensor,
                      self.passenger_state_sensor, self.destination_state_sensor, self.state_sensor,reward_sensor]




        action_one_behavior = BehaviorShell(plannerPrefix=self.prefix,name="Action0",index=0)

        action_two_behavior = BehaviorShell(plannerPrefix=self.prefix,name="Action1",index=1)

        action_three_behavior = BehaviorShell(plannerPrefix=self.prefix,name="Action2",index=2)

        action_four_behavior = BehaviorShell(plannerPrefix=self.prefix,name="Action3",index=3)

        action_five_behavior = BehaviorShell(plannerPrefix=self.prefix,name="Action4",index=4)

        action_six_behavior = BehaviorShell(plannerPrefix=self.prefix,name="Action5",index=5)

        # conditions
        is_at_pickup_location = LocationCondition(sensor=self.passenger_state_sensor,activator= ThresholdActivator(4),sensors=state_list,is_pick_up=True)
        is_at_dropboff_location = LocationCondition(sensor=self.passenger_state_sensor,activator= ThresholdActivator(4),
                                                  sensors=state_list, is_pick_up=False)

        has_passenger_condition = Condition(self.passenger_state_sensor, ThresholdActivator(4))

        at_pos_one_condition = Conjunction(self.get_equal_condition(self.row_state_sensor, 0)
                                           , self.get_equal_condition(self.col_state_sensor, 0))

        at_pos_two_condition = Conjunction(self.get_equal_condition(self.row_state_sensor, 0)
                                           , self.get_equal_condition(self.col_state_sensor, 4))

        at_pos_three_condition = Conjunction(self.get_equal_condition(self.row_state_sensor, 4)
                                             , self.get_equal_condition(self.col_state_sensor, 0))

        at_pos_four_condition = Conjunction(self.get_equal_condition(self.row_state_sensor, 4)
                                            , self.get_equal_condition(self.col_state_sensor, 3))

        at_location = Disjunction(at_pos_four_condition, at_pos_one_condition, at_pos_three_condition,
                                  at_pos_two_condition)

        dest_one_cond = Condition(self.destination_state_sensor, ThresholdActivator(0))
        row_one_cond = Condition(self.row_state_sensor, ThresholdActivator(0))
        col_one_cond = Condition(self.col_state_sensor, ThresholdActivator(0))
        pass_one_cond = Condition(self.passenger_state_sensor, ThresholdActivator(0))# to get the dest in the input

        action_one_behavior.addPrecondition(dest_one_cond)  # to get the dest in the input
        action_one_behavior.addPrecondition(row_one_cond)  # to get the row in the input
        action_one_behavior.addPrecondition(col_one_cond)  # to get the col in the input
        action_one_behavior.addPrecondition(pass_one_cond)  # to get the pass in the input

        action_six_behavior.addPrecondition(is_at_dropboff_location)
        action_six_behavior.addPrecondition(has_passenger_condition)

        action_five_behavior.addPrecondition(is_at_pickup_location)
        action_five_behavior.addPrecondition(Negation(has_passenger_condition))


        is_in_goal_state = Condition(self.state_sensor, ThresholdActivator(thresholdValue=15,name="state"))  # for getting the state of the game

        good_pick_off = Condition(reward_sensor,
                                  ThresholdActivator(thresholdValue=20,name="good"))  # successful dropoff
        illegal_action = Condition(reward_sensor, ThresholdActivator(thresholdValue=-10, isMinimum=False,name="illegal")) # wrong action

        #timestep_action = Conjunction(Negation(good_pick_off),Negation(illegal_action)) #everything else
        aboveminusone = Condition(reward_sensor,
                                  ThresholdActivator(thresholdValue=-1,name="above"))
        belowminusone = Condition(reward_sensor,
                                  ThresholdActivator(thresholdValue=-1,isMinimum=False,name="belo"))
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
        #no_hole = GoalBase(name="no_hole_goal", permanent=True,
        #                    conditions=[Negation(start_state_cond)], priority=1,
        #                    plannerPrefix=self.prefix)
        direct_to_goal_effect = Effect(sensor_name=is_in_goal_state.getFunctionNames()[0], indicator=1.0,
                               sensor_type=float)#

        action_two_behavior.correlations.append(direct_to_goal_effect)
        action_one_behavior.correlations.append(direct_to_goal_effect)
        action_three_behavior.correlations.append(direct_to_goal_effect)
        action_four_behavior.correlations.append(direct_to_goal_effect)

        direct_to_goal_effect2 = Effect(sensor_name=good_pick_off.getFunctionNames()[0], indicator=1.0,
                                       sensor_type=float)  #

        action_two_behavior.correlations.append(direct_to_goal_effect2)
        action_one_behavior.correlations.append(direct_to_goal_effect2)
        action_three_behavior.correlations.append(direct_to_goal_effect2)
        action_four_behavior.correlations.append(direct_to_goal_effect2 )

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

        self.test_env = TestTaxiEnv(self.env, self.manager, state_list)