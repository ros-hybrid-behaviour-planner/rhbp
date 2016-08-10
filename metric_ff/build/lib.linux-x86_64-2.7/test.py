#!/usr/bin/python2
import ff

with open("../../../metric_ff/robotDomain.pddl", "r") as infile:
    domain = "".join(infile.readlines())

with open("../../../metric_ff/robotProblem.pddl", "r") as infile:
    problem = "".join(infile.readlines())

result = ff.plan(domain, problem, searchMode=4)


print result

