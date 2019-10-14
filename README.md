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
pip install --user lindypy

```

Required for rhbp_rl

```bash
pip install --user h5py

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
@INPROCEEDINGS{hrabia2017goaldriven,
author={Christopher-Eyk Hrabia, Stephan Wypler and Sahin Albayrak},
booktitle={2017 3nd International Conference on Control, Automation and Robotics (ICCAR)},
title={Towards Goal-driven Behaviour Control of Multi-Robot Systems},
year={2017},
}
```
About the self-organisation extension:

```
Combining Self-Organisation with Decision-Making and Planning
Christopher-Eyk Hrabia, Tanja Katharina Kaiser, Sahin Albayrak
In: Multi-Agent Systems and Agreement Technologies: 15th European Conference, EUMAS 2017, and 5th International Conference, AT 2017, Evry, France, December 14-15, 2017; 2018
```

```bibtex
@InProceedings{hrabia2018combining,
author="Hrabia, Christopher-Eyk
and Kaiser, Tanja Katharina
and Albayrak, Sahin",
editor="Belardinelli, Francesco
and Argente, Estefan{\'i}a",
title="Combining Self-Organisation with Decision-Making and Planning",
booktitle="Multi-Agent Systems and Agreement Technologies",
year="2018",
publisher="Springer International Publishing",
address="Cham",
pages="385--399",
isbn="978-3-030-01713-2"
}
```

About the reinforcement learning extension:

```
Increasing Self-Adaptation in a Hybrid Decision-Making and Planning System with Reinforcement Learning
Christopher-Eyk Hrabia , Patrick Marvin Lehmann, Sahin Albayrak
In: 2019 IEEE 43rd Annual Computer Software and Applications Conference (COMPSAC); 2019
```

```bibtex
@INPROCEEDINGS{hrabia2019increasing, 
author={Christopher-Eyk {Hrabia} and Patrick Marvin {Lehmann} and Sahin {Albayrak}}, 
booktitle={2019 IEEE 43rd Annual Computer Software and Applications Conference (COMPSAC)}, 
title={Increasing Self-Adaptation in a Hybrid Decision-Making and Planning System with Reinforcement Learning}, 
year={2019}, 
volume={1}, 
number={}, 
pages={469-478}, 
keywords={decision-making, planning, reinforcement learning, self-adaptation, autonomous robots}, 
doi={10.1109/COMPSAC.2019.00073}, 
ISSN={0730-3157}, 
month={July},}
```

About the application of RHBP in the German SpaceBot Cup competition:

```
An autonomous companion UAV for the SpaceBot Cup competition 2015
Christopher-Eyk Hrabia, Martin Berger, Axel Hessler, Stephan Wypler, Jan Brehmer, Simon Matern and Sahin Albayrak
In: Robot Operating System (ROS) - The Complete Reference (Volume 2); 2017
```

```bibtex
@InBook{hrabia2017ros,
  Title                    = {Robot Operating System (ROS) - The Complete Reference (Volume 2)},
  Author                   = {Christopher-Eyk Hrabia and Martin Berger and Axel Hessler and Stephan Wypler and Jan Brehmer and Simon Matern and Sahin Albayrak},
  Chapter                  = {An autonomous companion UAV for the SpaceBot Cup competition 2015},
  Editor                   = {Anis Koubaa},
  Publisher                = {Springer International Publishing},
  Year                     = {2017},
  Doi                      = {10.1007/978-3-319-54927-9},
}
```

About the application in the Multi-Agent Programming Contest (MAPC) (https://github.com/agentcontest)

```
Applying robotic frameworks in a simulated multi-agent contest
Christopher-Eyk Hrabia, Patrick Marvin Lehmann, Nabil Battjbuer, Axel Heßler, Sahin Albayrak
In: Annals of Mathematics and Artificial Intelligence; 2018
```

```bibtex
@Article{hrabia2018mapc,
author="Hrabia, Christopher-Eyk
and Lehmann, Patrick Marvin
and Battjbuer, Nabil
and Hessler, Axel
and Albayrak, Sahin",
title="Applying robotic frameworks in a simulated multi-agent contest",
journal="Annals of Mathematics and Artificial Intelligence",
year="2018",
month="May",
day="07",
abstract="In the Multi-Agent Programming Contest 2017 the TUBDAI team of the Technische Universit{\"a}t Berlin is using the complex multi-agent scenario to evaluate the application of two frameworks from the field (multi-)robot systems. In particular the task-level decision-making and planning framework ROS Hybrid Behaviour Planner (RHBP) is used to implement the execution and decision-making for single agents. The RHBP framework builds on top of the framework Robot Operating System (ROS) that is used to implement the communication and scenario specific coordination strategy of the agents. The united team for the official contest is formed by volunteering students from a project course and their supervisors.",
issn="1573-7470",
doi="10.1007/s10472-018-9586-x",
url="https://doi.org/10.1007/s10472-018-9586-x"
}
```

About the application in the BMBF EffFeu project:

```
EffFeu Project: Towards Mission-Guided Application of Drones in Safety and Security Environments
Christopher-Eyk Hrabia, Axel Heßler, Yuan Xu, Jacob Seibert, Jan Brehmer, and Sahin Albayrak
In: MDPI Sensors, Special Issue Unmanned Aerial Vehicle Networks, Systems and Applications; 2019
```

```bibtex
@Article{hrabia2019efffeu,
AUTHOR = {Hrabia, Christopher-Eyk and Hessler, Axel and Xu, Yuan and Seibert, Jacob and Brehmer, Jan and Albayrak, Sahin},
TITLE = {EffFeu Project: Towards Mission-Guided Application of Drones in Safety and Security Environments},
JOURNAL = {Sensors},
VOLUME = {19},
YEAR = {2019},
NUMBER = {4},
ARTICLE-NUMBER = {973},
URL = {http://www.mdpi.com/1424-8220/19/4/973},
ISSN = {1424-8220},
DOI = {10.3390/s19040973}
}
```

