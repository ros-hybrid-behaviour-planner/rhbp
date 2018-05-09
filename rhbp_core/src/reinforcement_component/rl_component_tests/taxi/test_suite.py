"""
this file evaluates the diffeerent algorithm , exploration techniques and action selections
"""
import matplotlib.pyplot as plt

import numpy
import pandas

from reinforcement_component.rl_component_tests.taxi.dqn.taxi_dqn_new_negative_state import TaxiTestConditionsNew
from reinforcement_component.rl_component_tests.taxi.dqn.taxi_dqn_normal import  TaxiTestNormal, TaxiTestConditions, TaxiTestDecoded


def plot_for_different_seed(num_range,env,threshold,should_print):
    tuples = []
    for i in range(1, num_range+1):
        tuple = env.start_env(num_prints=100, threshold=threshold, random_seed=i, should_print=should_print)
        tuples.append(tuple)
    for index in range(len(tuples)):
        plt.plot(tuples[index][:, 1], tuples[index][:, 0], marker="o", label=str(tuples[index][0,3])+" + seed= "+str(index))
    plt.xlabel("time in steps")
    plt.ylabel("reward")
    plt.legend(loc=4)
    plt.show()
    for index in range(len(tuples)):
        plt.plot(tuples[index][:, 2], tuples[index][:, 0], marker="o", label=str(tuples[index][0,3])+" + seed= "+str(index))
    plt.xlabel("time in episodes")
    plt.ylabel("reward")
    plt.legend(loc=4)
    plt.show()

def compare_classes(envs,num_range,threshold,should_print):
    tuples = []
    for num_env in range(len(envs)):
        for i in range(1, num_range+1):
            tuple = envs[num_env].start_env(num_prints=100, threshold=threshold, random_seed=i, should_print=should_print)
            tuples.append(tuple)

    for index in range(len(tuples)):
        plt.plot(tuples[index][:, 1], tuples[index][:, 0], marker="o", label=str(tuples[index][0,3])+"+ seed= "+str(index/len(envs)))
        plt.plot(tuples[index][:, 1], tuples[index][:, 0], marker="o",
                 label=str(tuples[index][0, 3]) + "+ seed= " + str(index / len(envs)))
    plt.xlabel("time in steps")
    plt.ylabel("reward")
    plt.legend(loc=4)
    plt.show()
    for index in range(len(tuples)):
        plt.plot(tuples[index][:, 2], tuples[index][:, 0], marker="o", label=str(tuples[index][0,3])+"+ seed= "+str(index/len(envs)))
    plt.xlabel("time in episodes")
    plt.ylabel("reward")
    plt.legend(loc=4)
    plt.show()


if __name__ == '__main__':
    dqn = 0
    qlearning = 1

    test_case_normal = TaxiTestNormal(algorithm=dqn)

    test_case_cond = TaxiTestConditions(algorithm=dqn)

    test_case_normal_q = TaxiTestNormal(algorithm=qlearning)

    test_case_cond_q = TaxiTestConditions(algorithm=qlearning)

    test_case_decoded = TaxiTestDecoded(algorithm=dqn)

    test_case_decoded_q = TaxiTestDecoded(algorithm=qlearning)

    test_case_new_cond = TaxiTestConditionsNew(algorithm=dqn)
    #plot_for_different_seed(4,test_case,0,False)
    #plot_for_different_seed(5, test_case_cond, 0, False)
    plot_for_different_seed(1, test_case_new_cond, 0, True)
    #compare_classes([test_case_normal,test_case_cond,test_case_normal_q,test_case_cond_q,test_case_decoded#
    #                 ],1,0,False)
