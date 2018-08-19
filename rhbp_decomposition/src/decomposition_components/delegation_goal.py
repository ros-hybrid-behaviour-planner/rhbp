
from decomposition_components.delegation_clients import RHBPDelegationClient
from decomposition_components.goal_wrapper import RHBPGoalWrapper, DecompositionGoal
from delegation_components.delegation_manager import DelegationManager


class RunningGoalWrapper(RHBPGoalWrapper):

    def __init__(self, name, goal, conditions, satisfaction_threshold=1.0):
        super(RunningGoalWrapper, self).__init__(name=name, conditions=conditions, satisfaction_threshold=satisfaction_threshold)
        self._goal = goal

    def __del__(self):
        del self._conditions

    def send_goal(self, name=""):
        self._goal.start_with_contractor(planner_prefix=name)
        self._created_goal = True

    def terminate_goal(self):
        self._created_goal = False
        self._goal.terminate_goal()


class DelegationGoal(DecompositionGoal):

    def __init__(self, name, delegation_manager, permanent=False, conditions=None, plannerPrefix="", priority=0, satisfaction_threshold=1.0, activated=True):
        self._contractor_found = False
        super(DelegationGoal, self).__init__(name=name, permanent=permanent, conditions=conditions,
                                             plannerPrefix=plannerPrefix, priority=priority,
                                             satisfaction_threshold=satisfaction_threshold, activated=activated)
        self._client = RHBPDelegationClient()
        self._client.register(delegation_manager=delegation_manager)
        self._goal_wrapper = None
        self._delegation_id = -1

    def _init_services(self):

        if not self._contractor_found:
            return

        print "starting services"
        super(DelegationGoal, self)._init_services()
        print "done"

    def register(self):

        if not self._contractor_found:
            return

        super(DelegationGoal, self).register()

    def start_with_contractor(self, planner_prefix):
        self._planner_prefix = planner_prefix
        self._contractor_found = True
        self._init_services()
        self.register()

    def terminate_goal(self):
        if hasattr(self, '_registered') and self._registered:
            self.unregister(terminate_services=True)

    def start_auction(self):
        self._goal_wrapper = RunningGoalWrapper(name=self.name, goal=self,
                                                conditions=self._conditions,
                                                satisfaction_threshold=self.satisfaction_threshold)
        self._delegation_id = self._client.delegate_goal_wrapper(goal_wrapper=self._goal_wrapper,
                                                                 known_depth=0)

    def finish_auction(self):
        for i in range(DelegationManager.DEFAULT_AUCTION_STEPS):
            self._client.do_step()

