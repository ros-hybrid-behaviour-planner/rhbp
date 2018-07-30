import matplotlib
import rospy

matplotlib.use('agg')
import matplotlib.pyplot as plt

import numpy
import pandas as pd

from taxi_dqn_new_negative_state import TaxiTestConditionsNew, \
    TaxiTestAllConditionsNew
from taxi_dqn_normal import TaxiTestNormal, TaxiTestConditions, \
    TaxiTestDecoded, TaxiTestDecodedCond


def compare_classes(envs, num_range, threshold, should_print):
    """
    runs the given environments and plots the results
    :param envs: list of environments
    :param num_range: number of times each environment should be run
    :param threshold: stop environment until the average results reach this threshold
    :param should_print: True if each step should be printed
    :return: 
    """
    tuples = []
    df = pd.DataFrame(columns=["reward", "steps", "episodes", "name"])
    # start environments save results
    for num_env in range(len(envs)):
        for i in range(1, num_range + 1):
            tuple = envs[num_env].start_env(num_prints=100, threshold=threshold, random_seed=i,
                                            should_print=should_print)
            tuple[:, 0] = numpy.round(tuple[:, 0].astype(numpy.double).astype(numpy.int), 0)

            tuples.append(tuple)
            df_new = pd.DataFrame(tuple, columns=["reward", "steps", "episodes", "name"])
            df = df.append(df_new)
    # uncomment for reading data from a csv and plotting them
    """ 
    df = pd.read_csv("experiment_taxi_df"+str(0)+".csv",names=["reward","steps","episodes","name"])
    df = df[df.name != "dqn0"]
    df = df[df.name != "dqn1"]
    """
    # cast data types
    df["reward"] = df["reward"].astype(int)
    df["episodes"] = df["episodes"].astype(int)
    df["reward"] = df["reward"].astype(str).astype(int)
    df["steps"] = df["steps"].astype(str).astype(int)
    df["episodes"] = df["episodes"].astype(str).astype(int)
    df = df[df.episodes % 100 == 0]
    df = df.drop("steps", 1)
    # change names
    change_names_dict = {"dqn0": "dqn", "test_agentTaxiTestAllConditionsNew0": "Hot_State_Encoding_Conditions",
                         "test_agentTaxiTestDecoded0": "Variable_Encoding",
                         "test_agentTaxiTestNormal0": "Hot_state_Encoding",
                         "test_agentTaxiTestDecodedCond0": "Variable_Encoding_Conditions",
                         "rhbp_variable_encoding": "RHBP_Variable_Encoding", "taxi_rhbp": "RHBP_Hot_State_Encoding"}
    df = df.replace({"name": change_names_dict})
    ax = None
    # print each algorithm class grouped by episodes
    for algorithm in df.name.unique():
        grouped_df = df.groupby(["name"]).get_group(algorithm).groupby("episodes")
        average_df = grouped_df.mean()
        std_df = grouped_df.std()["reward"].values.tolist()
        unstack = average_df.unstack()
        average_df = average_df.rename(index=str, columns={"reward": algorithm})
        if ax is None:
            ax = average_df.plot(legend=True, marker="o", yerr=std_df)
            ax.legend()
        else:
            ax = average_df.plot(legend=True, marker="o", yerr=std_df, ax=ax)
            ax.legend()
    # make plot nicer and safe it
    plt.xticks(numpy.arange(0, 101, 10), numpy.arange(0, 10001, 1000))
    plt.legend(loc=0)
    plt.ylabel("reward")
    print("plotted")
    plt.savefig("experiment_taxi_graph_episodes.png")


if __name__ == '__main__':
    dqn = 0
    filename = "taxi_experiment_results.csv"

    test_case_normal = TaxiTestNormal(algorithm=dqn)

    test_case_cond = TaxiTestConditions(algorithm=dqn)

    test_case_decoded = TaxiTestDecoded(algorithm=dqn)

    test_case_new_cond = TaxiTestConditionsNew(algorithm=dqn)

    test_case_new_all_cond = TaxiTestAllConditionsNew(algorithm=dqn)

    test_case_decoded_cond = TaxiTestDecodedCond(algorithm=dqn)

    compare_classes([test_case_new_all_cond], 1, 0, True)
