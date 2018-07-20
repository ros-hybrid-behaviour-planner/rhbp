
from delegation_components.cost_evaluators import CostEvaluatorBase
from delegation_components.delegation_errors import DelegationPlanningWarning
from decomposition_components.delegation_behaviour import DelegationBehaviour

import utils.rhbp_logging
rhbplogger = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.delegation')


class PDDLCostEvaluator(CostEvaluatorBase):
    """
    Cost function evaluator for DelegationManager using the symbolic
    PDDL-Planer of the RHBP-Manager
    """

    TASK_CAPACITY_FACTOR = 1
    WORKLOAD_PROPORTION_FACTOR = -0.025
    ADDITIONAL_WORKLOAD_FACTOR = 1
    ADDITIONAL_DEPTH_FACTOR = 1
    COOPERATION_AMOUNT_FACTOR = 1
    CONTRACTOR_NUMBER_FACTOR = 0.1

    def __init__(self, manager):
        """
        Constructor

        :param manager: Manager that the cost is computed with
        :type manager: Manager
        """

        super(PDDLCostEvaluator, self).__init__()
        self._manager = manager

    def compute_cost_and_possibility(self, goal_representation, current_task_count, max_task_count, current_depth, max_depth, members, own_name):
        """
        Computes cost and possibility of a goal given its statement.
        This is accomplished by trying to plan with the PDDL-planner of the
        manager and using the plans to extract possibility and cost

        :param goal_representation: PDDL representation of the goal (goal-statement)
        :type goal_representation: str
        :param current_task_count: number of tasks currently running
        :type current_task_count: int
        :param max_task_count: maximum number of tasks
        :type max_task_count: int
        :param current_depth: current depth of this task
        :type current_depth: int
        :param max_depth: maximum depth for tasks
        :type max_depth: int
        :param members: list of the names of current members of this delegation
        :type members: list(str)
        :param own_name: name of this unit (like the names in members)
        :type own_name: str
        :return: Cost, Possibility for this goal
        :rtype: float, bool
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
            self._last_cost = self._compute_cost(full_plan=full_plan, simple_plan=simple_plan,
                                                 task_count=current_task_count, max_task_count=max_task_count,
                                                 depth=current_depth, max_depth=max_depth,
                                                 members=members, own_name=own_name)

        except Exception as e:  # catch any exception
            self._last_possibility = False
            raise DelegationPlanningWarning(str(e.message))

        return self._last_cost, self._last_possibility

    def _compute_cost(self, full_plan, simple_plan, task_count, max_task_count, depth, max_depth, members, own_name):
        """
        Extract all needed information out of the plan and computes cost

        :param full_plan: PDDL plan for the goal and all currently active goals
        :type full_plan: PDDL plan (dict)
        :param simple_plan: PDDL plan for just the goal
        :type simple_plan: PDDL plan (dict)
        :param task_count: number of tasks currently running
        :type task_count: int
        :param max_task_count: maximum number of tasks
        :type max_task_count: int
        :param depth: current depth of this task
        :type depth: int
        :param max_depth: maximum depth for tasks
        :type max_depth: int
        :param members: list of the names of current members of this delegation
        :type members: list(str)
        :param own_name: name of this unit (like the names in members)
        :type own_name: str
        :return: the cost that was computed
        """

        base_steps, full_steps, simple_steps = self._get_plan_steps(full_plan, simple_plan)

        num_delegations = self._determine_number_of_delegations(simple_plan)
        new_delegations = 1.0 if num_delegations > 0 else 0.0

        new_contractor = 0.0 if own_name in members else 1.0
        contractor_count = float(len(members))

        task_capacity_utilization = (1 + self.TASK_CAPACITY_FACTOR * float(task_count) / float(max_task_count))
        workload_proportion = (1 + self.WORKLOAD_PROPORTION_FACTOR * simple_steps / full_steps)
        additional_workload = (1 + self.ADDITIONAL_WORKLOAD_FACTOR * base_steps / full_steps)
        additional_depth = (1 + self.ADDITIONAL_DEPTH_FACTOR * new_delegations * float(depth) / float(max_depth))
        cooperation_amount = (1 + self.COOPERATION_AMOUNT_FACTOR * num_delegations / simple_steps)
        contractor_number = (1 + self.CONTRACTOR_NUMBER_FACTOR * new_contractor * (1 - contractor_count / (depth + 1)))

        cost = full_steps * task_capacity_utilization * workload_proportion * additional_workload * additional_depth * cooperation_amount * contractor_number

        rhbplogger.loginfo(own_name+": Computed cost for accomplishing the goal from following factors:"
                           + "\n\tSteps of full plan "+str(full_steps)
                           + "\n\tTask Capacity Utilization "+str(task_capacity_utilization)
                           + "\n\tWorkload Proportion "+str(workload_proportion)
                           + "\n\tAdditional Workload "+str(additional_workload)
                           + "\n\tAdditional Delegation Depth "+str(additional_depth)
                           + "\n\tCooperation Amount "+str(cooperation_amount)
                           + "\n\tNumber of Contractors "+str(contractor_number)
                           + "\nResulting Cost (product of the above): "+str(cost))

        return cost

    def _get_plan_steps(self, full_plan, simple_plan):
        """
        Returns the number of steps for all important PDDL-plans (base_plan =
        plan without a new goal; full_plan = plan with the new goal;
        simple_plan = plan for just the single new goal)

        :param full_plan: PDDL plan for the goal and all currently active goals
        :type full_plan: PDDL plan (dict)
        :param simple_plan: PDDL plan for just the goal
        :type simple_plan: PDDL plan (dict)
        :return: Number of steps of the base_plan, the full_plan and the
                simple_plan
        :rtype: float, float, float
        """

        base_plan = self._manager.plan
        if base_plan and "cost" in base_plan and base_plan["cost"] != -1.0:
            base_steps = base_plan["cost"]
        else:
            base_steps = 0.0
        full_steps = full_plan["cost"]
        simple_steps = simple_plan["cost"]
        return base_steps, full_steps, simple_steps

    def _determine_number_of_delegations(self, plan):
        """
        Determines the number of distinct DelegationBehaviours in the given plan and
        returns it. For a plan without "actions" it returns 0.

        :param plan: a PDDL plan
        :type plan: PDDL plan (dict)
        :return: number of distinct used DelegationBehaviours
        :rtype: float
        """

        if not (plan and "actions" in plan):
            return 0.0

        delegation_behaviour_names = [b.name for b in self._manager.behaviours if b.behaviour_type == DelegationBehaviour.TYPE_STRING]
        actions = plan["actions"]
        used_delegation_behaviours = set([b for b in actions.values() if b in delegation_behaviour_names])
        num_delegations = float(len(used_delegation_behaviours))
        return num_delegations
