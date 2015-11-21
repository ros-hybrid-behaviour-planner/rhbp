#!/usr/bin/python2
import ff
with open("../../robotDomain.pddl", 'r') as infile:
    domain = "".join(infile.readlines())
    
with open("../../robotProblem.pddl", 'r') as infile:
    problem = "".join(infile.readlines())
    
print ff.plan(domain, problem, searchMode=4)

