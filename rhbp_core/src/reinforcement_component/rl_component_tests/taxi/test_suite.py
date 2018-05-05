"""
this file evaluates the diffeerent algorithm , exploration techniques and action selections
"""
import matplotlib.pyplot as plt

import numpy
import pandas

from reinforcement_component.rl_component_tests.taxi.dqn.taxi_dqn_normal import TaxiTestSuiteDQN



if __name__ == '__main__':

    dqn = 0
    qlearning = 1

    test_case = TaxiTestSuiteDQN(algorithm=dqn)

    tuples1 = test_case.start_env(num_prints=100, threshold=0, random_seed=0, should_print=True)
    plt.plot(tuples1[:,2],tuples1[:,0],marker="o",color="black")

    tuples2 =  test_case.start_env(num_prints=100, threshold=0, random_seed=1, should_print=True)
    plt.plot(tuples1[:, 2], tuples1[:, 0], marker="o", color="red")

    plt.show()