#!/usr/bin/python2
import ffp

with open("../metric_ff/robotDomain.pddl", "r") as infile:
    domain = "".join(infile.readlines())

with open("../metric_ff/robotProblem.pddl", "r") as infile:
    problem = "".join(infile.readlines())


plan = ffp.plan(domainPDDL=domain,
                problemPDDL=problem,
                searchMode=5,
                upperCostBound=1000,
                weight=5,
                costMinimization=True,
                debug=0)

print("Plan Result:", plan)

