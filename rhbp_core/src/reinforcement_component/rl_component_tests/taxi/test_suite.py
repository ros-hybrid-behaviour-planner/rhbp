"""
this file evaluates the diffeerent algorithm , exploration techniques and action selections
"""
import matplotlib.pyplot as plt

import numpy
import pandas

from reinforcement_component.rl_component_tests.taxi.dqn.taxi_dqn_normal import TaxiTestSuiteDQN, \
    TaxiTestSuiteDQNConditions


def plot_for_different_seed(num_range,env,threshold,should_print):
    tuples = []
    for i in range(1, num_range+1):
        tuple = env.start_env(num_prints=100, threshold=threshold, random_seed=i, should_print=should_print)
        tuples.append(tuple)
    for index in range(len(tuples)):
        plt.plot(tuples[index][:, 1], tuples[index][:, 0], marker="o", label=str(tuples[index][0:3])+" + seed= "+str(index))
    plt.xlabel("time in steps")
    plt.ylabel("reward")
    plt.legend()
    plt.show()
    for index in range(len(tuples)):
        plt.plot(tuples[index][:, 2], tuples[index][:, 0], marker="o", label=str(tuples[index][0:3])+" + seed= "+str(index))
    plt.xlabel("time in episodes")
    plt.ylabel("reward")
    plt.legend()
    plt.show()

def compare_classes(envs,num_range,threshold,should_print):
    tuples = []
    for num_env in range(len(envs)):
        for i in range(1, num_range+1):
            tuple = envs[num_env].start_env(num_prints=100, threshold=threshold, random_seed=i, should_print=should_print)
            tuples.append(tuple)
    for index in range(len(tuples)):
        plt.plot(tuples[index][:, 1], tuples[index][:, 0], marker="o", label=str(tuples[index][0:3])+"+ seed= "+str(index/len(envs)))
    plt.xlabel("time in steps")
    plt.ylabel("reward")
    plt.legend()
    plt.show()
    for index in range(len(tuples)):
        plt.plot(tuples[index][:, 2], tuples[index][:, 0], marker="o", label=str(tuples[index][0:3])+"+ seed= "+str(index/len(envs)))
    plt.xlabel("time in episodes")
    plt.ylabel("reward")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    dqn = 0
    qlearning = 1

    test_case = TaxiTestSuiteDQN(algorithm=dqn)

    test_case_cond = TaxiTestSuiteDQNConditions(algorithm=dqn)

    test_case_q = TaxiTestSuiteDQN(algorithm=qlearning)

    test_case_cond_q = TaxiTestSuiteDQNConditions(algorithm=qlearning)
    #plot_for_different_seed(4,test_case,0,False)
    #plot_for_different_seed(5, test_case_cond, 0, False)
    #plot_for_different_seed(5, test_case_cond, 8, False)
    compare_classes([test_case_cond,test_case,test_case_q,test_case_cond_q],5,0,True)
