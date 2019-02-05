# RHBP - ROS Hybrid Behaviour Planner

RHBP is a planning and decision making component for ROS. The hybrid approach is formed by a reactive behaviour network (package **rhbp_core**) and a symbolic planner (package **metric_ff**).

## Packages

1. **[rhbp_core](rhbp_core/README.md):** All core components of the package. It includes all user required classes in order to model the application in a behaviour-based style. More details can be found in the [package](rhbp_core/README.md).
2. **[rhbp_utils](rhbp_utils/README.md):** Additional helpful components for the application of RHBP, e.g. knowledge sensors for the knowledge base). Please see [package](rhbp_utils/README.md) for more details.
3. **[metric_ff](metric_ff/README.md):** The PDDL-compatible symbolic planner that is used to provide long-term guidance to the behaviour network. The planner is used by rhbp_core.
4. **ffp:** Just a wrapper for metric_ff allowing to run the planning process in a separate child process in order to avoid existing memory issues of the original metric_ff implementation.
5. **[knowledge_base](knowledge_base/README.md):** A versatile ROS knowledge base implementation based on the tuple space implementation lindypy. Further documentation inside the [package](knowledge_base/README.md)
6. **[rhbp_decomposition](rhbp_decomposition/README.md):** RHBP extension that enables automated task decomposition, allocation and delegation. Please see [package](rhbp_decomposition/README.md) for more details.
6. **[rhbp_rl](rhbp_rl/README.md):** RHBP extension that integrates reinforcement learning as additional activation source. Please see [package](rhbp_rl/README.md) for more details.

## Build

### Dependencies

Required for knowledge_base

```bash
pip install lindypy

```

### Clone and build

In order to use this package in your ROS application just clone this repository into your workspace sources and build your workspace.

```bash
cd src
git clone --recursive https://github.com/DAInamite/rhbp
```

Build everything

```bash
cd ..
catkin_make
```

## Usage
From a user perspective you will only depend on the packages **[rhbp_core](rhbp_core/README.md)**, which completely covers all interaction with the planner from you, and possibly on optional extensions like **[rbhp_utils](rhbp_utils/README.md)**. Please refer to the READMEs of the corresponding package for further information about how to use them.

## Citation
If you want to refer to our work in scientific context, please cite the following articles.

For the core implementation:

```
Towards Goal-driven Behaviour Control of Multi-Robot Systems
Christopher-Eyk Hrabia, Stephan Wypler and Sahin Albayrak
In: 2017 IEEE 3nd International Conference on Control, Automation and Robotics (ICCAR); 2017
```

```bibtex
@INPROCEEDINGS{hrabia_goal_driven_2017,
author={Christopher-Eyk Hrabia, Stephan Wypler and Sahin Albayrak},
booktitle={2017 3nd International Conference on Control, Automation and Robotics (ICCAR)},
title={Towards Goal-driven Behaviour Control of Multi-Robot Systems},
year={2017},
}
```

For application of RHBP in the German SpaceBot Cup competition:

```
An autonomous companion UAV for the SpaceBot Cup competition 2015
Christopher-Eyk Hrabia, Martin Berger, Axel Hessler, Stephan Wypler, Jan Brehmer, Simon Matern and Sahin Albayrak
In: Robot Operating System (ROS) - The Complete Reference (Volume 2); 2017
```

```bibtex
@InBook{hrabia_ros_2017,
  Title                    = {Robot Operating System (ROS) - The Complete Reference (Volume 2)},
  Author                   = {Christopher-Eyk Hrabia and Martin Berger and Axel Hessler and Stephan Wypler and Jan Brehmer and Simon Matern and Sahin Albayrak},
  Chapter                  = {An autonomous companion UAV for the SpaceBot Cup competition 2015},
  Editor                   = {Anis Koubaa},
  Publisher                = {Springer International Publishing},
  Year                     = {2017},
  Doi                      = {10.1007/978-3-319-54927-9},
}
```
