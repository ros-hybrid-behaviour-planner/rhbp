#!/usr/bin/python2
import ffp
with open("/home/stephan/catkinWS/robotDomain.pddl", "r") as infile:
    domain = "".join(infile.readlines())

with open("/home/stephan/catkinWS/robotProblem.pddl", "r") as infile:
    problem = "".join(infile.readlines())

print ffp.plan(domain, problem)

