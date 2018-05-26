import pandas
import matplotlib.pylab as plt


class Analysis(object):
    """
    A simple behaviour for triggering generic MAPC actions that just need a type and static parameters
    """

    def __init__(self,filename = "experiment_results.csv",range_start=0, range_end = -1):
        self.filename = filename
        self.range_start = range_start
        self.range_end = range_end

    def plot_left_over_time(self):
        try:
            df = pandas.read_csv(self.filename)
        except IOError:
            print("file not found")
            return
        #print("jo")
        #print(df)
        if self.range_end < 0:
            self.range_end =len(df)
        df = df[self.range_start:self.range_end]
        df.plot(y="l")
        plt.show()

