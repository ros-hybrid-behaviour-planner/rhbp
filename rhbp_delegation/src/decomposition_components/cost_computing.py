"""
The PDDLCostEvaluator that uses the PDDL-Planer of the RHBP for the plans

@author: Mengers
"""

from delegation_components.cost_evaluators import CostEvaluatorBase, CostParameters
from delegation_components.delegation_errors import DelegationPlanningWarning
from decomposition_components.delegation_behaviour import DelegationBehaviour
import utils.rhbp_logging
# DelegationLogger for rhbplogger
rhbplogger = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.delegation')


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

    def _plan_and_extract_parameters(self, goal_representation, own_name, members, depth, max_depth, task_count, max_task_count):
        """
        Makes the plans using the planer and determines the parameters needed
        for the cost function

        :param goal_representation: representation of the goal
        :type goal_representation: PDDL goal-statement
        :param own_name: name of the corresponding DelegationManager
        :type own_name: str
        :param members: list of current members of this delegation
        :type members: list
        :param depth: current depth of this delegation
        :type depth: int
        :param max_depth: max depth for this DelegationManager
        :type max_depth: int
        :param task_count: current task count for this DelegationManager
        :type task_count: int
        :param max_task_count: max task count for this DelegationManager
        :type max_task_count: int
        :return: extracted parameters for the cost function
        :rtype: CostParameters
        """

        # Make plans
        full_plan = self._manager.plan_with_additional_goal(goal_statement=goal_representation)
        simple_plan = self._manager.plan_this_single_goal(goal_statement=goal_representation)
        # Check if planned successful
        if not (full_plan and "cost" in full_plan and full_plan["cost"] != -1.0):
            raise DelegationPlanningWarning("Full plan unsuccessful!")
        if not (simple_plan and "cost" in simple_plan and simple_plan["cost"] != -1.0):
            raise DelegationPlanningWarning("Simple plan unsuccessful!")

        base_steps, full_steps, simple_steps = self._get_plan_steps(full_plan, simple_plan)

        num_delegations = self._determine_number_of_delegations(simple_plan)
        new_delegations = 1.0 if num_delegations > 0 else 0.0

        new_contractor = 0.0 if own_name in members else 1.0
        contractor_count = len(members)

        parameters = CostParameters(name=own_name, base_steps=base_steps, full_steps=full_steps,
                                    simple_steps=simple_steps, depth=depth, max_depth=max_depth,
                                    new_contractor=new_contractor, contractor_count=contractor_count,
                                    new_delegations=new_delegations, num_delegations=num_delegations,
                                    task_count=task_count, max_task_count=max_task_count)

        return parameters

    def _log_cost_computed(self, name, cost, full_steps, task_capacity_utilization, workload_proportion,
                           additional_workload, additional_depth, cooperation_amount, contractor_number):
        """
        Logs the information about the cost computed

        :param name: name of this DelegationManager
        :type name: str
        :param cost: cost computed
        :type cost: float
        :param full_steps: steps of the full plan
        :type full_steps: int
        :param task_capacity_utilization: task_capacity_utilization-factor value
        :type task_capacity_utilization: float
        :param workload_proportion:workload_proportion-factor value
        :type workload_proportion: float
        :param additional_workload:additional_workload-factor value
        :type additional_workload: float
        :param additional_depth:additional_depth-factor value
        :type additional_depth: float
        :param cooperation_amount:cooperation_amount-factor value
        :type cooperation_amount: float
        :param contractor_number:contractor_number-factor value
        :type contractor_number: float
        """

        rhbplogger.loginfo(name + ": Computed cost for accomplishing the goal from following factors:"
                           + "\n\tSteps of full plan " + str(full_steps)
                           + "\n\tTask Capacity Utilization " + str(task_capacity_utilization)
                           + "\n\tWorkload Proportion " + str(workload_proportion)
                           + "\n\tAdditional Workload " + str(additional_workload)
                           + "\n\tAdditional Delegation Depth " + str(additional_depth)
                           + "\n\tCooperation Amount " + str(cooperation_amount)
                           + "\n\tNumber of Contractors " + str(contractor_number)
                           + "\nResulting Cost (product of the above): " + str(cost))

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
        :rtype: int
        """

        if not (plan and "actions" in plan):
            return 0.0

        delegation_behaviour_names = [b.name for b in self._manager.behaviours if b.behaviour_type == DelegationBehaviour.TYPE_STRING]
        actions = plan["actions"]
        used_delegation_behaviours = set([b for b in actions.values() if b in delegation_behaviour_names])
        num_delegations = len(used_delegation_behaviours)
        return num_delegations
