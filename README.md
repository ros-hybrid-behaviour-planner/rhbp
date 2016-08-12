# RHBP - ROS Hybrid Behaviour Planner

RHBP is a planning and decision making component for ROS. The hybrid approach is formed by a reactive behaviour network (package rhbp_core) and a symbolic planner (package metric_ff).

## Packages

* **rhbp_core:** The core of the component. It includes all user required classes in order to model the application in a behaviour-based style. More details can be found in the package README.md
* **metric_ff:** The PDDL-compatible symbolic planner that is used to provide long-term guidance to the behaviour network. The planner is used by rhbp_core
* **ffp:** Just a wrapper for metric_ff allowing to run the planning process in a separate child process in order to avoid existing memory issues of the original metric_ff implementation

