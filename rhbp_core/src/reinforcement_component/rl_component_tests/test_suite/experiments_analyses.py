import pandas
import matplotlib.pylab as plt


class Analysis(object):
    """
    A simple behaviour for triggering generic MAPC actions that just need a type and static parameters
    """

    def __init__(self,filename = "experiment_results.csv"):
        self.filename = filename


    def plot_left_over_time(self):
        try:
            df = pandas.read_csv(self.filename)
        except IOError:
            print("file not found")
            return
        print("jo")
        print(df)
        df.plot(y="l")
        plt.show()

