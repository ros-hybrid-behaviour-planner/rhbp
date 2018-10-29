# rhbp_decomposition
This package provides a functionality for explicit dynamic cooperation of agents/robots.

## Usage
* Use the expanded [Manager](src/decomposition_components/managers.py)
    instead of the Manager of the [rhbp_core](../rhbp_core) 
    if the agent should be able to receive tasks from others and therefore
    participate in auctions
* these Managers need to be allowed to participate then (*participate_in_auctions()*)
* afterwards [DelegationBehaviours](src/decomposition_components/delegation_behaviour.py),   
    and [DelegableBehaviours](src/decomposition_components/delegation_behaviour.py)
    can be used at all agents (also agents without other kinds of Managers)
    to model the capacities of different agents/robots in the system and delegate
    tasks to them if the capacities are needed
* Also the [DelegationGoal](src/decomposition_components/delegation_goal.py)
    can be used as a Goal that will automatically determine the best suiting participating
    Manager in the system
    
For additional information see code documentation in form of doc-strings.

## Concept
The different additional behaviours are used to include the capacities of different
robots in the plans of other robots.
This way robots can cooperate explicitly. If the capacities of a different robot
should be used (a Behaviour is activated) the [DelegationModule](TODO) will be used to start
an auction. Then this auction determines the best suiting robot to take the task.
While the task is delegated, it will be ensured that the other robot is still active.

For additional information on the auction see the [DelegationModule](TODO) documentation.

## Configuration
The package can be configured mainly at the [DelegationModule](TODO), where timeout-durations,
cost-function parameters etc. can be configured to suit your needs. 