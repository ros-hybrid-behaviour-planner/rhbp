from delegation_components.goal_wrappers import GoalWrapperBase
from behaviour_components.goals import GoalBase
from delegation_components.delegation_errors import DelegationError


class RHBPGoalWrapper(GoalWrapperBase):
    """
    Class that wraps goals that are used in the RHBP.
    Uses PDDL-based goal representations and sends goals
    by registering them at the corresponding manager.
    """

    def __init__(self, name, conditions, satisfaction_threshold=1.0):
        """
        Constructor

        :param name: name of the goal
        :param conditions: list of conditions of this goals
        :param satisfaction_threshold: satisfaction threshold
                of the goal
        """

        super(RHBPGoalWrapper, self).__init__(name=name)

        self._conditions = conditions
        self._satisfaction_threshold = satisfaction_threshold

    def __del__(self):
        """
        Destructor
        """

        super(RHBPGoalWrapper, self).__del__()
        del self._conditions

    def get_goal_representation(self):
        """
        Returns the representation of the goal in form
        of a PDDL goal-statement

        :return: goal representing PDDL-String
        """

        return " ".join([x.getPreconditionPDDL(self._satisfaction_threshold).statement for x in self._conditions])

    def send_goal(self, name=""):
        """
        Sends the goal to the RHBP-manager with this planner-prefix
        by registering it directly at the manager

        :param name: prefix of the manager, that should
                receive the goal
        :raises DelegationError: if there is any problem with the goal creation
        """
        try:
            self._goal = GoalBase(name=self.goal_name(), plannerPrefix=name, conditions=self._conditions, satisfaction_threshold=self._satisfaction_threshold)
            self._created_goal = True
            return
        except Exception as e:
            self._created_goal = False
            self._goal = None
            raise DelegationError("Sending goal raised exception: " + e.message)

    def terminate_goal(self):
        """
        Terminates (unregister and deletes) the created goal
        """

        if self.goal_is_created():
            self._created_goal = False
            self._goal.__del__()   # unregisters goal
            self._goal = None
