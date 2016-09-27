############################
# BEHAVIOUR_PLANNER MANUAL #
############################

A behaviour planner works by periodically spreading activation in a network of behaviours and goals.
Behaviours accumulate activation every iteration (and lose it over time with a factor called activationDecay to ensure their activation is indeed caused by the current situation and not by events in the past). 
Executable behaviours with high-enough activation (above the global activationThreshold of the planner) and satisfied preconditions (above their individual readyThreshold) are started (Well, there is more to it - keep reading).
When they finish executing their activation is reset to 0 to give other behaviours a chance.

##################################
# Important Behaviour Properties #
##################################

Before we continue with how the activation of each behaviour gets calculated at every iteration (up to the point necessary to know for a developer) and how the interface between the planner and behaviours must look like (more interesting to a developer) we must have a look at other properties of a behaviour not relevant for activation calculation but for the planner to decide whether it can start (or must interrupt) a particular behaviour.
Those properties are:
    - Whether it is activated (users may deativate a behaviour to exclude it from the planning process).
    - What its effects are. This is expressed by the behaviour's correlations: a dictionary with (sensor) names as keys and an inticator as value. The value range is real and the allowed interval is [-1, 1]. Interpretations of this value are very intuitive in some situations and sometimes a bit harder: When the affected (sensor) value is ordinal then a positive correlation value indicates that the sensor value will rise when the behaviour executes and vice versa for a neative correlation value. In cases where there is no order only the positive part of the scale is used to express the influence of the behaviour on this property. For example, a behaviour that moves the robot has influence on the pose so its correlation to the pose sensor is 1. The correlations are used by the planner to detect conflicts that would arise when multiple behaviours that affect the same properties would operate at the same time.
    - Whether it is interruptable. This is important in combination with the following attribute (priority).
    - Its priority. The priority of a behaviour is a positive integer starting at 0 (default). The higher the value the more important the behaviour is. In situations where multiple executable behaviours have conflicting correlations the one with highest priority interrupts those with less priority if and only if the inferior behaviours are interruptable.
    - A progress indicator. The value range is real, [0, 1] and may be used later when the behaviour planner is part of a hybrid systems. Behaviour implementers are requested to implement a progress estimation into their behaviours even though it is not used at the moment because.

###########################################
# Start Conditions Checked by the Planner #
###########################################

The previous section indicates that the planner actually checks more than just the activation value before launching a behaviour.
At first the behaviours are sorted by activation in decending order. The the planner examines every behaviour and checks the following conditions:
    1) If a behaviour is active. Acitve means antivated and there have been no internal issues that could render the behaviour inactive.
    2) If it is already executing. If so, it won't get started again
    3) If it is executable (precondition satisfaction larger than readyThreshold)
    4) If its activation is high enough (activation larger than activationThreshold)
    5) If it does not conflict with any running behaviour or if the conflict is solveable by interruption of all conflicting behaviour (of course only works in case the behaviour to be started has the highest priority of all of them and all inferiour behaviours are interruptable)

The activationThrehold is decreased by the factor activationThresholdDecay at every iteration when no behaviour is executing but there are active behaviours available. Likewise, the threshold is increased by the same amount when a behaviour is started. This should prevent the system from starvation and adjust the threshold to the level of activation available in the system.

##########################
# Activation Calculation #
##########################

Activation basically comes from 3 different sources: The situation, the behaviour constellation and goals. Note that this also works in the opposite way and a conflict will introduce negative activation that reduces the overall sum.
Activation by the situation comes from the preconditions and is in the range [0, 1]. If there are no preconditions then it can run in any situations so this value is 1. If there are at least one precondition then the activation by the situation is the average satisfaction of each precondition (also [0 to 1]).
Because behaviour may have precondition they want them to be fulfilled. They express these desires with a collection of wishes. Wishes are similar to correlations and that is no coincidence: A wish is a tuple of a (sensor) name and an indicator with a comparable meaning to the indicator of correlations. Let's say a rechargeBehaviour has a precondition saying that the "batterySensor" should read 5V or less and the current battery has 6V. In this situation it would have a wish for the "batterySensor" with a negative indicator value (meaning that it wants the value to fall).
Consider a UAV as another example: The explorationBehaviour wants the aircraft to fly at a certain height. Right at the beginning this explorationBehaviour has a wish for the "altitudeSensor" to read e.g. 2m but the UAV is still on the ground so the sensor reports 0m and the wish indicates 1 for the "altitudeSensor" (desire to rise). Now it happens that there is a startBehviour that is correlated with the "altitudeSensor" in a positive way because it lifts the UAV up in the air. This startBehaviour can fulfill a precondition of the explorationBehaviour and is therefor called a predecessor of the explorationBehaviour. The explorationBehaviour is therefore consequently the successor of the startBehaviour and the landBehaviour having a negative correlation the the "altitudeSensor" would be a conflictor to the explorationbehaviour.
Now that the relationships created by correlations and preconditions (via wishes) are established we have a network of behaviours. Within this network activation is fed backwards from a successor to a predecessor if it can make a a precondition of the successor come true. Executable behaviours spread activation forward to their successors because if they, as predecessors, are executed it is likely that also the successors will be executed. If a behaviour conflicts with another its activation will be reduced.
The last component in the network are goals. Goals incorporate conditions and are either permanent or one-time. Non-permanent goals vanish once they are achieved. Like behaviours, goals express their desires by wishes and the thus influence the spreading of activation the just the same way as behaviours do: Aactivation is given to behaviours that are positively correlated to the sensors contained in the goal's conditions or taken away otherwise.

########################################################
# Planner - Behaviour and Planner - Goal Communication #
########################################################

The planner and all goals and behaviours are decoupled and use ROS Services to communicate. The planner itself offers services called 'AddBehaviour', 'AddGoal', 'RemoveBehaviour', 'RemoveGoal', and 'ForceStart'. Each of them is prefixed with the planner's prefix so that multiple planners can run simultaneously. If the plannerPrefix is for example 'sim' then the service to add a behaviour has the name 'simAddBehaviour'.
Behaviours and goals also have to offer services to interact with the planner. For a behaviour those services are 'GetStatus', 'Start', 'Stop', 'Activate', and 'Priority', each prefixed with the behaviour's name. The 'GetStatus' service is polled by the planner periodically to update its internal state. The planner uses 'Start' and 'Stop' to execute and interrupt the behaviour, respectively.
Goals must offer the 'Activate' and 'GetStatus' service (again, prefixed with the goal's name).

#########################################
# Writing Your Own Behaviours and Goals #
#########################################

Base classes for behaviours and goals do all the necessary service registration and offer a lot of functionality that can be used to get started quickly.
The constructor of the BehaviourBase takes only the behaviour's name as mandatory argument but offers to configure all parameter of the behaviour via convenient keyword arguments. Almost all properties can also be adjusted later. The GoalBase's constructor can also take all necessary parameters all at once. Please see the well commented __init__() methods and examples for further information.
The planner only requires that the services operate as expected and report the needed values in the correct format.
For the behaviour, only the following methods need to be implemented so that the automatically registered 'GetStatus' service callback can send the values back to the planner:
computeActivation() 
computeSatisfaction()
self.computeWishes()
self.getProgress()
Please see the doxygen documentation for details.

The default implementations of the above methods try to reduce the effort to a minimum: These implementations (except for getProgress, of course) are designed to operate on the precondition list (specifiable in the constructor) if those preconditions offer the right interface.
A condition must have provide the 2 properties satisfaction (and return a float [0, 1]) and optional (bool) and the method getWishes which must return a list of (name, indicator) tuples.
Besides a regular Condition object that wraps an Activator object there are the pseudo-conditions Conjunction and Disjunction the take multiple regular conditions, Conjunctions, and Disjunctions and compute the corresponding logical expressions of them.
Sensors must offer a value property and are identified by a name. It is important that the Sensors' names are consistent in the entire system e.g. in wishes and correlations. Sensors may represent the actual state of the world or system and subscribe to ROS topics for example (there is a SimpleTopicSensor that does exactly that). But they could also represent anything else like an intermediate value or flag not measurable in the world. Sensors can be used for information exchange among behaviours. This is especially meaningful when this information affects another behaviours activation or execution.
Activator wrap a Sensor and a desired value (or value range or anything) and abstract those arbitrary types and values to a activation value from 0 to 1 using their activation function. They also offer the feature to compute a wish-indicator from the activator's internal activation function and the current sensor's value. There are a couple of simple activation functions pre-implemented that work with boolean, int and real sensor values: A BooleanActivator for a desired boolean value, a ThresholdActivator that implements a lower or upper boundary, and a LinearActivator where the activation rises linear within an interval.
More complex sensor values may either be simplified to the typles supported by the given Activators or new Activators matching those types can be created. There is no "right" way to do it and the preferred solution depends heavily on the situation and type.
Every behaviour has a start() and stop() method that gets called by the corresponding services and should switch the behaviour's activity on or off. As every service handler in the behaviour_planner it must not block. The actionlib is a good way to further decouple the behaviour's actual functionality from the management part.

Goals are very similar to behaviours in the way that they have a list of conditions that define their goal. Everything works exactly like in behaviours except they have no activation, correlations, and start() and stop() methods but additionally a isPermanent flag.

###########
# Example #
###########

This section shows some examples of the functionalities and concepts and how to use them. The snippets are taken from the current behaviour_test package (except the planner (Manager) instantiation) located in the workspace at https://gitlab.tubit.tu-berlin.de/s.wypler/catkinWS and consist of 4 behaviours: a startBehaviour, a landBehaviour, a moveBehaviour and a goHomeBehaviour.
The objective is the following: Start the UAV, move arround to explore the world, come home when the battery is empty, land at home and stay there.

The planner itself is created using just a few lines of code:

import rospy
from behaviour_components.managers import Manager

if __name__ == '__main__':
    m = Manager(activationThreshold = 21, prefix = "sim") # The same prefix must be passed to the behaviours and goals that should register themselves at this planner instance. ActivationThreshold is more or less arbitrary as it is self adjusting as described above; it should not be too low.
    rate = rospy.Rate(1) # 1Hz: It does not make sense for the planner to plan faster than the fastest behaviour duration or expected environmental changes
    while(True):
        m.step()
        rate.sleep()

Now that we have a planner we can set up the senario:

import testBehaviours # in here are all the behaviours and custom sensors
import rospy
from behaviour_components.goals import GoalBase # this is the base class to create a goal
from behaviour_components.conditions import Condition
from behaviour_components.activators import LinearActivator

if __name__ == '__main__':
    rospy.init_node('test_sim')
    # creating sensors
    heightSensor = testBehaviours.SonarSensor("heightSensor", "/robot/sonar", True)
    # setting up conditions
    isFlying = Condition(heightSensor, LinearActivator(0, .7), name = "isFlyingCondition") # the first argument of the linear activator marks the value with 0 activation, the second the value with full activation. Values out of that range are clamped to 0 and 1
    isNotFlying = Condition(heightSensor, LinearActivator(1, .3), name = "isNotFlyingCondition") # it also works in reverse
    # behaviours 
    startBehaviour = testBehaviours.StartBehaviour(plannerPrefix = "sim") # Note that the planner prefix is consistent
    goHomeBehaviour = testBehaviours.GoHomeBehaviour(plannerPrefix = "sim")
    landBehaviour = testBehaviours.LandBehaviour(plannerPrefix = "sim")
    moveBehaviour = testBehaviours.MoveBehaviour(plannerPrefix = "sim")
    # setting up goals
    startedGoal = GoalBase("started", conditions = [isFlying], plannerPrefix = "sim") # Goals may take a list of conditions and all must be true at once. in this case there is only one condition (but it must still be a list).
    landedGoal = GoalBase("landed", conditions = [isNotFlying], plannerPrefix = "sim")
    rospy.loginfo("running")
    rospy.spin()

Now we can have a look at the SonarSensor which shows how a complex message format can be simplified to a single float value.

from sensor_msgs.msg import Range
from behaviour_components.sensors import SimpleTopicSensor

class SonarSensor(SimpleTopicSensor):
    """
    This class extracts the height value of a sensor_msgs.msg.Range message
    """
    def __init__(self, name, topic, createLog = False):
        super(SonarSensor, self).__init__(name, topic, Range, createLog) # as all Sensors it takes a name as argument. The SimpleTopicSensor additionally needs the topic to subscribe to, the data type published at the topic and optionally a flag indicating that it should create a log file dor debug purposes. 
        self.__messageCounter = 0
        
    def subscriptionCallback(self, msg):
        self.update(msg.range) # this is the only line that does actual work here. It extracts the message and provides the plain value.
        if self._iShouldCreateLog and self.__messageCounter > 50:
            #self._logFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._value))
            self._logFile.flush()
            self.__messageCounter = 0
        self.__messageCounter += 1

We are now ready to look at an entire behaviour. We chose the LandBehaviour for this purpose because it uses both custom Sensors and Activators.

from behaviour_components.behaviours import BehaviourBase
< further import statements >

class LandBehaviour(BehaviourBase):
    '''
    classdocs
    '''
    def __init__(self, **kwargs):
        '''
        Constructor
        '''
        #rospy.init_node('testBehaviourNode', anonymous=True, log_level=rospy.INFO) # If it was a separate node we would comment this line out but this example runs inside another test node.
        super(LandBehaviour, self).__init__("landBehaviour", **kwargs) # as mentioned above the name is required and all other parameter are optional.
        self._heightSensor = SonarSensor("heightSensor", "/robot/sonar")
        self._poseSensor = PoseSensor("poseSensor", "/position_controller/pose") # The PoseSensor will be explained later as it shows how to extract a complex type from a topic
        self._batterySensor = BatterySensor("batterySensor")
        self._home = Pose()
        self._home.position.x = 0.0
        self._home.position.y = 0.0
        self._correlations = {'heightSensor': -1.0}
        self._preconditions = [
                               Condition(self._heightSensor, LinearActivator(0, .3), name = "isFlyingCondition"),
                               Condition(self._poseSensor, DistanceActivator(self._home), name = "atHomeConditionL"), # The DistanceActivated will also be explained later because it demonstrates how to work with complex types.
                               Condition(self._batterySensor, LinearActivator(14.0, 12.8), name = "fullBatteryCondition")
                              ]
    
    def start(self):
        rospy.loginfo("########### LAND NOW ###########")
        rospy.wait_for_service("/position_controller/set_altitude")
        setAltitudeService = rospy.ServiceProxy("/position_controller/set_altitude", SetAltitude)
        result = setAltitudeService(0.0)
        rospy.loginfo("called setAltitude(0.0) with result: %s", result) 
        
    def getStatus(self, request):
        if self._isExecuting: # the startCallback inherited from BehaviourBase sets this attribute to true when it starts the behaviour.
            if self._heightSensor.value > .3: # the ground level is not 0!
                rospy.loginfo("### still landing ###")
            else:
                rospy.loginfo("########### reached ground ###########")
                self._isExecuting = False # A behaviour must switch its _isExecuting attibute off when it is finished.
        return super(LandBehaviour, self).getStatus(request) # Actually, it is not good to misuse the getStatus callback for internal housekeeping like progress monitoring and logging. The actual behaviour's implemetations should do this and the getStatus callback should remain untouched.

Here is the PoseSensor which does simlipfy the topic's mesage but it is still more complex than bool, int or float.

< includes >

class PoseSensor(SimpleTopicSensor):
    """
    This class extracts the pose value of a nav_msgs.msg.Odometry message
    """
    def __init__(self, name, topic, createLog = False):
        super(PoseSensor, self).__init__(name, topic, Odometry, createLog)
        
    def subscriptionCallback(self, msg):
        self.update(msg.pose.pose) # although this is only a part of the message the result is still a complex type and we need a suitable Activator to deal with it.
        
Lastly here is the DistanceActivator that can operate on the PoseSensor's values. Note that there are no higher or lower Poses. For the wishes it only makes sense to indicate whether the current pose is OK or not (so all the Activator cares about is the Distance of the current Pose to the desired Pose).

< includes >

class DistanceActivator(LinearActivator):
    '''
    This class is an activator that compares a geometry_msgs.msg.Pose to a desired Pose
    '''
    def __init__(self, desiredValue, zeroActivationDistance = 1.0, fullActivationDistance = 0.1, minActivation = 0, maxActivation = 1):
        '''
        Constructor
        '''
        super(DistanceActivator, self).__init__(zeroActivationDistance, fullActivationDistance, minActivation, maxActivation)
        self._desired = desiredValue # this is the desired Pose
        self._zeroActivationDistance = zeroActivationDistance
        self._fullActivationDistance = fullActivationDistance
    
    def computeActivation(self, value):
        """
        computes a linear slope based on distance
        """
        assert isinstance(value, Pose)
        return super(DistanceActivator, self).computeActivation(self._distance(value))
    
    def getWish(self, value):
        """
        return whether there is a desire to move
        """
        assert isinstance(value, Pose)
        return abs(super(DistanceActivator, self).getWish(self._distance(value)))
        
    def _distance(self, p):
        """
        computes Manhattan distance 
        cares only about distance in x-y-direction
        """
        return abs(p.position.x - self._desired.position.x) + abs(p.position.y - self._desired.position.y)