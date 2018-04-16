import tensorflow as tf
import numpy

from rhbp_core.msg import SensorValue


class SensorValueTransformer:

    def __init__(self):
        #self.manager = manager
        #print("init transformer")
        self.conditions={}

    def get_value_of_condition(self,cond):
        value = None
        #print("get_value")
        try:
            #print("value1:",cond._sensor.value,cond._sensor._name)
            value = float(cond._sensor.value)
            sensor_value = SensorValue()
            sensor_value.name = cond._sensor._name
            sensor_value.value = value
            sensor_value.encoding = cond._sensor.rl_extension.encoding
            sensor_value.state_space = cond._sensor.rl_extension.state_space
            sensor_value.include_in_rl = cond._sensor.rl_extension.include_in_rl
            #print(sensor_value)
            return sensor_value
        except Exception as e:
            #print(e)
            value = None
        try:

            value = float(cond._condition._sensor.value)
            #print("value2:", value,cond._condition._sensor._name)
            sensor_value = SensorValue()
            sensor_value.name = cond._condition._sensor._name
            sensor_value.value = value
            sensor_value.encoding = cond._condition._sensor.rl_extension.encoding
            sensor_value.state_space = cond._condition._sensor.rl_extension.state_space
            sensor_value.include_in_rl = cond._condition._sensor.rl_extension.include_in_rl
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


class InputStateTransformer:

    def __init__(self,manager):
        self._manager = manager

    def calculate_reward(self):
        """
                this function calculates regarding the fulfillment and priorities of the active goals
                a reward value. 
                :return: 
                """
        # TODO think about better logic maybe
        # todo use wishes instead of fulfillment
        reward_value = 0
        for goal in self._manager.activeGoals:
            # print(goal,goal.fulfillment,goal.priority)
            # goal_value = goal.fulfillment * (10 ** goal.priority)
            goal_value = goal.fulfillment * goal.priority
            reward_value += goal_value

        return reward_value

    def behaviour_to_index(self,name):
        num = 0
        for b in self._manager.behaviours:
            if b == name:
                return num
            num += 1
        return None

    def make_hot_state_encoding(self,state,num_state_space):
        state = int(state)
        #print(state,num_state_space)
        array = numpy.identity(num_state_space)[state:state + 1]
        return numpy.identity(num_state_space)[state:state + 1].reshape([num_state_space,1])

    def transform_input_values(self):
        """
        this function uses the wishes and sensors to create the input vectors
        :return: input vector
        """
        # TODO transform like strings or similar to other values . e.g. hot-state-encoding (give sensor choice of encoding)
        # init input array with first row of zeros
        input_array = numpy.zeros([1,1])
        # extend array with input vector from wishes
        for behaviour in self._manager.behaviours:
            # check for each sensor in the goal wishes for behaviours that have sensor effect correlations
            for wish in behaviour.wishes:
                r=3#TODO include later again
                #wish_row = numpy.array([wish.indicator]).reshape([1,1])
                #input_array = numpy.concatenate([input_array,wish_row])
        # extend array with input vector from sensors
        # save which sensor were already included
        #TODO behavior get input called twice. getstatus
        sensor_input = {}
        for behaviour in self._manager.behaviours:
            # TODO include here wishes
            #print(behaviour)
            for sensor_value in behaviour.sensor_values:
                if not sensor_input.has_key(sensor_value.name):
                    #print("sensor vlaue input1", sensor_value)
                    if not sensor_value.include_in_rl:
                        continue
                    if sensor_value.encoding=="hot_state":
                        value = self.make_hot_state_encoding(sensor_value.value,sensor_value.state_space)
                    else:
                        value = numpy.array([[sensor_value.value]])
                    sensor_input[sensor_value.name] = value
                    #print(sensor_value.name,sensor_value.value,value)
                    input_array = numpy.concatenate([input_array, value])
        # cut out first row and return
        for goal in self._manager._goals:
            #print(goal.wishes)
            #print(goal,goal.fulfillment)
            for sensor_value in goal.sensor_values:
                #print("sensor_value",sensor_value)
                if not sensor_input.has_key(sensor_value.name):
                    #print("sensor vlaue input",sensor_value)
                    if not sensor_value.include_in_rl:
                        continue
                    if sensor_value.encoding=="hot_state":
                        value = self.make_hot_state_encoding(sensor_value.value,sensor_value.state_space)
                    else:
                        value = numpy.array([[sensor_value.value]])
                    #print(sensor_value.name,sensor_value.value,value)
                    #wish_row = numpy.array([[sensor_value.value]])
                    #value=wish_row
                    sensor_input[sensor_value.name] = value
                    #wish_row = numpy.array([[value]])
                    #print(numpy.array([[5]]))
                    input_array = numpy.concatenate([input_array, value])
        input_array = input_array[1:]
        #print(len(input_array),input_array)
        #print("input:",numpy.argmax(input_array))
        # TODO get wishes from sensors
        return input_array