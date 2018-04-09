import tensorflow as tf
import numpy as np

from rhbp_core.msg import SensorValue


class InputStateTransformer:

    def __init__(self):
        #self.manager = manager
        print("init transformer")
        self.conditions={}

    def get_value_of_condition(self,cond):
        value = None
        try:
            value = float(cond._sensor.value)
            sensor_value = SensorValue()
            sensor_value.name = cond._sensor._name
            sensor_value.value = value
            sensor_value.encoding = cond._sensor.encoding
            sensor_value.state_space = cond._sensor.state_space
            return sensor_value
        except Exception :
            value = None
        try:
            value = float(cond._condition._sensor.value)

            sensor_value = SensorValue()
            sensor_value.name = cond._condition._sensor._name
            sensor_value.value = value
            sensor_value.encoding = cond._condition.encoding
            sensor_value.state_space = cond._condition.state_space
            return sensor_value
        except Exception :
            value = None
        try:
            list = []
            for c in cond._conditions:
                value = self.get_value_of_condition(c)
                if value is not None:
                    list.append(self.get_value_of_condition(c))
            return list
        except Exception :
            value = None
        #print("no value found",cond)
        return None

    def get_values_of_list(self,full_list):
        # gets all values in list. the list can contain multiple lists
        new_list = []
        for ele in full_list:
            if isinstance(ele, (list,)):
                new_list.extend( self.get_values_of_list(ele))
            else:
                new_list.append(ele)
        return new_list

    def get_sensor_values(self,conditions):
        list_of_sensor_values = []
        #print("sensor", self.name,len(self.get_preconditions()  ))
        for p in conditions:
            value = self.get_value_of_condition(p)
            if isinstance(value,(list,)):
                list_of_sensor_values.extend(self.get_values_of_list(value))

            else:
                if(value is not None):
                    list_of_sensor_values.append(value)
        return list_of_sensor_values


    def get_current_state(self):
        # TODO implement connection to sensors too
        # TODO check then also for double preconditions because of negation or conjunction
        # TODO question: only conditions influencing this network behavior or from all
        for behaviour in self.manager.behaviours:
            behaviour.fetchState(0) # 0 is just dummy. Note: delete that a number is needed
            for cond in behaviour.condition_values:
                self.conditions[cond.name]=cond.activation

        state = np.zeros(len(self.conditions))
        index=0
        for k,v in self.conditions.iteritems():
            #print(k,v)
            state[index]=v
            index += 1

        #print(state,state.shape)
        state = state.reshape([1,len(self.conditions)])
        #print(state,state.shape)
        #print(np.identity(16)[0:0+1],np.identity(16)[0:0+1].shape)
        test=np.identity(4)[0:0 + 1]
        return state

    def get_last_action(self):
        return 0

    def get_reward_from_state(self):
        """
        this function calculates regarding the fulfillment and priorities of the active goals
        a reward value. it prioritizes the goals in  a way that a higher priority is always more important
        than a smaller one (power of 10)
        :return: 
        """
        # TODO think about better logic maybe

        reward_value = 100
        for goal in self.manager.get_goals():
            goal_value = goal.fulfillment * (10 ** goal.priority)
            reward_value += goal_value
        return reward_value

    def get_random_action(self):

        return np.random.randint(len(self.manager.get_behaviors())+1)