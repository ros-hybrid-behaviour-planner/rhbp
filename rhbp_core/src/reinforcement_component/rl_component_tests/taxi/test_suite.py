"""
this file evaluates the diffeerent algorithm , exploration techniques and action selections
"""
import matplotlib.pyplot as plt

from reinforcement_component.rl_component_tests.taxi.dqn.taxi_dqn_new_negative_state import TaxiTestConditionsNew, \
    TaxiTestAllConditionsNew
from reinforcement_component.rl_component_tests.taxi.dqn.taxi_dqn_normal import TaxiTestNormal, TaxiTestConditions, \
    TaxiTestDecoded
from src.rcs_ros_bridge.src.experiments_analyses import ExperimentAnalysis


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
    filename_goals = "../../../../../../../../../.ros/goal_experiment_results.csv"
    filename = "../../../../../../../../../.ros/experiment_results.csv"

    an = ExperimentAnalysis(filename=filename,filename_goals=filename_goals)
    an.plot_sequences(start=334,min_num=10)
    an.plot_left_over_time(start=64)
    exit(0)
    test_case_normal = TaxiTestNormal(algorithm=dqn)

    test_case_cond = TaxiTestConditions(algorithm=dqn)

    test_case_normal_q = TaxiTestNormal(algorithm=qlearning)

    test_case_cond_q = TaxiTestConditions(algorithm=qlearning)

    test_case_decoded = TaxiTestDecoded(algorithm=dqn)

    test_case_decoded_q = TaxiTestDecoded(algorithm=qlearning)

    test_case_new_cond = TaxiTestConditionsNew(algorithm=dqn)

    test_case_new_all_cond = TaxiTestAllConditionsNew(algorithm=dqn)
    #plot_for_different_seed(4,test_case,0,False)
    #plot_for_different_seed(5, test_case_cond, 0, False)
    plot_for_different_seed(5, test_case_decoded, 0, True)
    #compare_classes([test_case_normal,test_case_cond,test_case_normal_q,test_case_cond_q,test_case_decoded#
    #                 ],1,0,False)
