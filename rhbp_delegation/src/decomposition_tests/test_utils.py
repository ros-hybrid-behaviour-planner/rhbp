"""
Mocked Classes of the RHBP for testing purposes

@author: Mengers
"""

from copy import copy


class MockedManager(object):
    """
    Mocked RHBP-Manager
    """

    def __init__(self):
        self._prefix = "test_prefix"
        self.plan = dict()
        self.actions = dict()
        self.actions[0] = "DelegationBehaviour"
        self.actions[1] = "BehaviourBase"
        self.plan["actions"] = self.actions
        self.plan["cost"] = 2.0
        b1 = MockedBehaviour(name="DelegationBehaviour", b_type="Delegation")
        b2 = MockedBehaviour(name="BehaviourBase", b_type="Base")
        b3 = MockedBehaviour(name="DelegationBehaviour2", b_type="Delegation")
        self.behaviours = [b1, b2, b3]
        self.failing_plans = False
        self.plan_exception = False

    # noinspection PyUnusedLocal
    def plan_with_additional_goal(self, goal_statement):
        if self.plan_exception:
            raise Exception()
        if self.failing_plans:
            return dict()
        plan = dict()
        actions = copy(self.actions)
        actions[2] = "DelegationBehaviour2"
        actions[3] = "BehaviourBase"
        plan["actions"] = actions
        plan["cost"] = 4.0
        return plan

    # noinspection PyUnusedLocal
    def plan_this_single_goal(self, goal_statement):
        if self.plan_exception:
            raise Exception()
        if self.failing_plans:
            return dict()
        plan = dict()
        actions = dict()
        actions[0] = "BehaviourBase"
        actions[1] = "DelegationBehaviour2"
        plan["actions"] = actions
        plan["cost"] = self.plan["cost"]
        return plan

    @property
    def prefix(self):
        return self._prefix


class MockedBehaviour(object):
    """
    Basic mocked Behaviour
    """

    def __init__(self, name, b_type):
        self.name = name
        self.behaviour_type = b_type


class FunctionPointerTester(object):
    """
    A Tester object for basic Function pointers
    """

    def __init__(self):
        self._function_called = False

    def function(self):
        self._function_called = True

    @property
    def function_called(self):
        """
        Whether the function that is pointed to was called or not
        """

        return self._function_called


class MockedGoal(object):
    """
    Mocked Goal for testing
    """

    def __init__(self, name):
        self.name = name
