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


    for i in range(5):
        tuples1 = test_case.start_env(num_prints=100, threshold=8, random_seed=i, should_print=False)
        plt.plot(tuples1[:, 1], tuples1[:, 0], marker="o")
    plt.show()