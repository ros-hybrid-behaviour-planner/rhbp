
from delegation_components.cost_evaluators import CostEvaluatorBase
from delegation_components.delegation_errors import DelegationPlanningWarning
from decomposition_components.delegation_behaviour import DelegationBehaviour


class PDDLCostEvaluator(CostEvaluatorBase):
    """
    Cost function evaluator for DelegationManager using the symbolic
    PDDL-Planer of the RHBP-Manager
    """

    def __init__(self, manager):
        """
        Constructor

        :param manager: Manager that the cost is computed with
        :type manager: Manager
        """

        super(PDDLCostEvaluator, self).__init__()
        self._manager = manager

    def compute_cost_and_possibility(self, goal_representation, num_of_tasks=0):
        """
        Computes cost and possibility of a goal given its statement.
        This is accomplished by trying to plan with the PDDL-planing-function
        and using this plan to extract cost/possibility

        :param goal_representation: PDDL representation of the goal (goal-statement)
        :return: Cost, Possibility for this goal
        :raises DelegationPlanningWarning: if planning failed
        """

        self._last_possibility = False
        self._last_cost = -1
        try:
            # Make plans
            full_plan = self._manager.plan_with_additional_goal(goal_statement=goal_representation)
            simple_plan = self._manager.plan_this_single_goal(goal_statement=goal_representation)
            # Check if planned successful
            if not (full_plan and "cost" in full_plan and full_plan["cost"] != -1.0):
                return self._last_cost, self._last_possibility
            if not (simple_plan and "cost" in simple_plan and simple_plan["cost"] != -1.0):
                return self._last_cost, self._last_possibility

            self._last_possibility = True
            self._last_cost = self.__compute_cost(full_plan=full_plan, simple_plan=simple_plan)

        except Exception as e:  # catch any exception
            self._last_possibility = False
            raise DelegationPlanningWarning(str(e.message))

        return self._last_cost, self._last_possibility

    def __compute_cost(self, full_plan, simple_plan):
        """
        Extract all needed information out of the plan and
        computes cost

        :param full_plan: PDDL plan for the goal and all currently active goals
        :param simple_plan: PDDL plan for just the goal
        :return: the cost that was computed
        """

        base_plan = self._manager.plan
        if base_plan and "cost" in base_plan and base_plan["cost"] != -1.0:
            base_steps = base_plan["cost"]
        else:
            base_steps = 0

        full_steps = full_plan["cost"]
        simple_steps = simple_plan["cost"]

        num_delegations = self.determine_number_of_delegations(simple_plan)
        print(num_delegations)
        print simple_plan
        print(full_steps)
        print(simple_steps)
        print(base_steps)

        # TODO create smart heuristic for cost based on steps and possible other stuff

        return full_steps

    def determine_number_of_delegations(self, plan):
        delegation_behaviour_names = [b.name for b in self._manager.behaviours if b.behaviour_type == DelegationBehaviour.TYPE_STRING]
        actions = plan["actions"]
        used_delegation_behaviours = set([b for b in actions.values() if b in delegation_behaviour_names])
        num_delegations = len(used_delegation_behaviours)
        return num_delegations
