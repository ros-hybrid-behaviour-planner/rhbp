'''
Created on 14.02.2017

@author: hrabia
'''

from abc import ABCMeta, abstractmethod

import ffp


class Planner:
    """
    Abstract planner interface class
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def plan(self, domain_pddl, problem_pddl):
        """
        Runner planner with given PDDL setup
        :param domain_pddl: the PDDL domain string
        :param problem_pddl: the PDDL problem string
        :return: plan dictionary with a field 'costs' and a field 'actions'
                costs describes the costs of the plan, actions is a dictonary of indices(key) and action(value)
        """
        pass


class MetricFFSearchMode(object):
    """
    Metric FF search modes
    """
    EHC_H_BFS = 0               #Standard-FF: EHC+H then BFS (cost minimization: NO)
    BFS = 1                     #BFS (cost minimization: NO)
    BFS_H = 2                   #BFS+H (cost minimization: NO)
    A_STAR_WEIGHTED = 3         #Weighted A* (cost minimization: YES)
    A_STAR_EPSILON = 4          #A*epsilon (cost minimization: YES)
    EHC_H_A_STAR_EPSILON = 5    #EHC+H then A*epsilon (cost minimization: YES)


class MetricFF(Planner):

    def __init__(self, search_mode=MetricFFSearchMode.EHC_H_A_STAR_EPSILON, upper_bound=1000):
        self.search_mode = search_mode
        self.upper_bound = upper_bound
        self.cost_minimization = True
        self.weight = 5
        self.debug = 0

    def plan(self, domain_pddl, problem_pddl):
        return ffp.plan(domainPDDL=domain_pddl, problemPDDL=problem_pddl, searchMode=self.search_mode,
                        upperCostBound=self.upper_bound, weight=self.weight, costMinimization=self.cost_minimization,
                        debug=self.debug)
