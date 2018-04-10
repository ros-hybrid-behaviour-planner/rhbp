#!/usr/bin/env python2
# this is the main file of a rhbp agent instance
# it contains the agents variables, topics subscriber and publisher, as well as the regarding callback function
# Although the job_execution process is start in this file
from __future__ import division # force floating point division when using plain /
from agent_modules_tests import *
from behaviour_components.activators import BooleanActivator, StringActivator, ThresholdActivator, GreedyActivator, \
    LinearActivator, EqualActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition, MultiSensorCondition, Disjunction
from behaviour_components.sensors import *
from behaviour_components.goals import OfflineGoal,GoalBase
from reinforcement_component.rl_component_tests.test_suite.rhbp_agent_base import RhbpAgentBase
from rcs_ros_bridge.msg import SimStart,  GenericAction,PlayMode, Goals,Flags,Lines

import rospy



class FrozenLakeAgent(RhbpAgentBase):
    def __init__(self):
        super(FrozenLakeAgent, self).__init__()
        rospy.logdebug("RhbpAgent::init")


    def init_behaviors(self):
        """
        here we could also evaluate the msg_old in order to initialize depending on the role etc.
        :param msg:  the message
        :type msg: SimStart
        """
        self.environment_name = 'FrozenLake-v0'
        #self.environment_name = 'Taxi-v2'
        self.init_environment(self.environment_name)



        self.state_sensor = Sensor(name="StateSensor")
        reward_sensor = RewardSensor(name="RewardSensor",intervall=100)
        reward_sensor.update(0)
        #self.state_sensor2 = Sensor(name="StateSensor2")
        #self.state_sensor2.update(4)
        action_one_behavior = MakeActionBehavior(plannerPrefix=self.prefix,name="ActionOne",reward_sensor=reward_sensor,
                                                 action_index=0,state_sensor=self.state_sensor,environment=self.env)

        action_two_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionTwo",reward_sensor=reward_sensor,
                                                 action_index=1,state_sensor=self.state_sensor, environment=self.env)

        action_three_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionThree",reward_sensor=reward_sensor,
                                                 action_index=2,state_sensor=self.state_sensor, environment=self.env)

        action_four_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionFour",reward_sensor=reward_sensor,
                                                 action_index=3,state_sensor=self.state_sensor, environment=self.env)

        #action_five_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionFive",
        #                                         action_index=0,state_sensor=self.state_sensor, environment=self.env)
        #action_six_behavior = MakeActionBehavior(plannerPrefix=self.prefix, name="ActionSix",
        #                                         action_index=0,state_sensor=self.state_sensor, environment=self.env)

        #is_in_goal_state = Condition(self.state_sensor,EqualActivator(desiredValue=15))
        is_in_goal_state = Condition(self.state_sensor, ThresholdActivator(thresholdValue=15))
        is_in_good_state = Condition(reward_sensor,
                                     ThresholdActivator(thresholdValue=1))
        is_in_bad_state = Condition(reward_sensor,ThresholdActivator(thresholdValue=-1,isMinimum=False))
        #start_state_cond = Condition(self.state_sensor, EqualActivator(desiredValue=0))

        #hole_1 = Condition(self.state_sensor, EqualActivator(desiredValue=5))
        #hole_2 = Condition(self.state_sensor, EqualActivator(desiredValue=7))
        #hole_3 = Condition(self.state_sensor, EqualActivator(desiredValue=11))
        #hole_4 = Condition(self.state_sensor, EqualActivator(desiredValue=12))
        #is_in_hole = Disjunction(hole_1,hole_2,hole_3,hole_4)
        #is_in_goal_state2 = Condition(self.state_sensor2, BooleanActivator(desiredValue=14))
        #action_four_behavior.addPrecondition(is_in_goal_state2)

        the_goal = GoalBase(name="goal_the", permanent=True,
                                          conditions=[is_in_goal_state], priority=0,
                                          plannerPrefix=self.prefix)

        the_goal2 = GoalBase(name="goal_bad", permanent=True,
                            conditions=[is_in_bad_state], priority=-1,
                            plannerPrefix=self.prefix)
        the_goal3 = GoalBase(name="goal_good", permanent=True,
                             conditions=[is_in_good_state], priority=1,
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

        direct_to_goal_effect2 = Effect(sensor_name=is_in_bad_state.getFunctionNames()[0], indicator=-1.0,
                                       sensor_type=float)  #

        action_two_behavior.correlations.append(direct_to_goal_effect2)
        action_one_behavior.correlations.append(direct_to_goal_effect2)
        action_three_behavior.correlations.append(direct_to_goal_effect2)
        action_four_behavior.correlations.append(direct_to_goal_effect2 )

        direct_to_goal_effect3 = Effect(sensor_name=is_in_good_state.getFunctionNames()[0], indicator=1.0,
                                        sensor_type=float)  #

        action_two_behavior.correlations.append(direct_to_goal_effect3)
        action_one_behavior.correlations.append(direct_to_goal_effect3)
        action_three_behavior.correlations.append(direct_to_goal_effect3)
        action_four_behavior.correlations.append(direct_to_goal_effect3)


