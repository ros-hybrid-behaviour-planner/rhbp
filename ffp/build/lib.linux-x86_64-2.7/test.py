#!/usr/bin/python2
import ffp
with open("../../../metric_ff/robotDomain.pddl", "r") as infile:
    domain = "".join(infile.readlines())

with open("../../../metric_ff/robotProblem.pddl", "r") as infile:
    problem = "".join(infile.readlines())

print ffp.plan(domain, problem)

