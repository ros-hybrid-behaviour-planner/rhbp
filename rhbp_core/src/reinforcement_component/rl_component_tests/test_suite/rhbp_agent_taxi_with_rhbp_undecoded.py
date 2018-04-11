#!/usr/bin/env python2
# this is the main file of a rhbp agent instance
# it contains the agents variables, topics subscriber and publisher, as well as the regarding callback function
# Although the job_execution process is start in this file
from __future__ import division # force floating point division when using plain /
from agent_modules_tests import *
from behaviour_components.activators import BooleanActivator, StringActivator, ThresholdActivator, GreedyActivator, \
    LinearActivator, EqualActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition, MultiSensorCondition, Disjunction, Conjunction
from behaviour_components.sensors import *
from behaviour_components.goals import OfflineGoal,GoalBase
from reinforcement_component.rl_component_tests.test_suite.rhbp_agent_base import RhbpAgentBase
from rcs_ros_bridge.msg import SimStart,  GenericAction,PlayMode, Goals,Flags,Lines

import rospy



class TaxiAgentRhbpUndecoded(RhbpAgentBase):
    def __init__(self):
        super(TaxiAgentRhbpUndecoded, self).__init__()
        rospy.logdebug("RhbpAgent::init")


    def get_equal_condition(self,sensor,value):
        above = Condition(sensor,
                                  ThresholdActivator(thresholdValue=value,name="aboveactiv"))
        below = Condition(sensor,
                                  ThresholdActivator(thresholdValue=value, isMinimum=False,name="belowactiv"))
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

        self.locs = locs = [(0, 0), (0, 4), (4, 0), (4, 3)]

        self.state_sensor = Sensor("state",state_space=500)
        self.row_state_sensor = Sensor(name="RowStateSensor",state_space=5,include_in_rl=False)
        self.col_state_sensor = Sensor(name="ColStateSensor",state_space=5,include_in_rl=False)
        self.passenger_state_sensor = Sensor(name="PassengerStateSensor",state_space=5,include_in_rl=False)
        self.destination_state_sensor = Sensor(name="DestinationStateSensor",state_space=4,include_in_rl=False)

        # init sensor with variables for state 26
        self.row_state_sensor.update(0)
        self.col_state_sensor.update(1)
        self.passenger_state_sensor.update(1)
        self.destination_state_sensor.update(2)

        state_list = [self.row_state_sensor,self.col_state_sensor,
                      self.passenger_state_sensor,self.destination_state_sensor,self.state_sensor]
        #state_list = self.state_sensor
        reward_sensor = RewardSensor(name="RewardSensor",intervall=10)
        reward_sensor.update(0)

        # down
        action_one_behavior = MakeActionBehaviorTaxi(plannerPrefix=self.prefix,name="ActionZero",reward_sensor=reward_sensor,
                                                 action_index=0,state_sensor=state_list,environment=self.env)
        # up
        action_two_behavior = MakeActionBehaviorTaxi(plannerPrefix=self.prefix, name="ActionOne",reward_sensor=reward_sensor,
                                                 action_index=1,state_sensor=state_list, environment=self.env)
        # right
        action_three_behavior = MakeActionBehaviorTaxi(plannerPrefix=self.prefix, name="ActionTwo",reward_sensor=reward_sensor,
                                                 action_index=2,state_sensor=state_list, environment=self.env)
        # left
        action_four_behavior = MakeActionBehaviorTaxi(plannerPrefix=self.prefix, name="ActionThree",reward_sensor=reward_sensor,
                                                 action_index=3,state_sensor=state_list, environment=self.env)

        # pickup
        action_five_behavior = MakeActionBehaviorTaxi(plannerPrefix=self.prefix, name="ActionFour",reward_sensor=reward_sensor,
                                                 action_index=4,state_sensor=state_list, environment=self.env)
        # dropoff
        action_six_behavior = MakeActionBehaviorTaxi(plannerPrefix=self.prefix, name="ActionFive",reward_sensor=reward_sensor,
                                                 action_index=5,state_sensor=state_list, environment=self.env)

        # conditions

        has_passenger_condition = Condition(self.passenger_state_sensor,ThresholdActivator(4))

        at_pos_one_condition = Conjunction(self.get_equal_condition(self.row_state_sensor,0)
                                           ,self.get_equal_condition(self.col_state_sensor,0))

        at_pos_two_condition = Conjunction(self.get_equal_condition(self.row_state_sensor, 0)
                                           , self.get_equal_condition(self.col_state_sensor, 4))

        at_pos_three_condition = Conjunction(self.get_equal_condition(self.row_state_sensor, 4)
                                           , self.get_equal_condition(self.col_state_sensor, 0))

        at_pos_four_condition = Conjunction(self.get_equal_condition(self.row_state_sensor, 4)
                                           , self.get_equal_condition(self.col_state_sensor, 3))

        at_location = Disjunction(at_pos_four_condition,at_pos_one_condition,at_pos_three_condition,at_pos_two_condition)

        dest_one_cond = Condition(self.destination_state_sensor,ThresholdActivator(0)) # to get the dest in the input

        action_one_behavior.addPrecondition(dest_one_cond)
        action_six_behavior.addPrecondition(at_location)
        action_six_behavior.addPrecondition(has_passenger_condition)

        action_five_behavior.addPrecondition(at_location)


        state_cond = Condition(self.state_sensor,ThresholdActivator(0))
        state_goal = GoalBase(name="state_goal", permanent=True,
                             conditions=[state_cond], priority=0,
                             plannerPrefix=self.prefix)


        # rewards and goals
        good_pick_off = Condition(reward_sensor,
                                  ThresholdActivator(thresholdValue=20))  # successful dropoff
        illegal_action = Condition(reward_sensor, ThresholdActivator(thresholdValue=-10, isMinimum=False)) # wrong action

        #timestep_action = Conjunction(Negation(good_pick_off),Negation(illegal_action)) #everything else
        aboveminusone = Condition(reward_sensor,
                                  ThresholdActivator(thresholdValue=-1))
        belowminusone = Condition(reward_sensor,
                                  ThresholdActivator(thresholdValue=-1,isMinimum=False))
        timestep_action = Conjunction(aboveminusone, belowminusone)  # everything else


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
