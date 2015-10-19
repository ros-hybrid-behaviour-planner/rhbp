#! /usr/bin/python2
import re
import datetime
import sys

if len(sys.argv) < 2:
    print "usage: %s <logfile>" % sys.argv[0]
    sys.exit()
logfile = sys.argv[1]

activationRegex = re.compile(r'(\d+-\d+-\d+\s+\d+:\d+:\d+,\d+):\s+activation\s+of\s+([a-zA-Z]+)\s+after\s+this\s+step:\s+(\d+\.\d+)')
stepRegex = re.compile(r'###################################### STEP (\d+) ######################################')
thresholdRegex = re.compile(r'(\d+-\d+-\d+\s+\d+:\d+:\d+,\d+):\s+current\s+activation\s+threshold:\s+(\d+\.\d+)')

behaviours = {
    "threshold" : {} # well, this is no nehaviour but it is only a variable name anyway
}

with open("activationFlow.dat", 'w') as outfile:
    currentStep = 0
    with open(logfile, 'r') as infile:
        for line in infile.readlines():
            match = activationRegex.search(line)
            if match:
                print match.group(0)
                dt = datetime.datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S,%f")
                ts = (dt - datetime.datetime(1970, 1, 1)).total_seconds()
                if match.group(2) in behaviours:
                    behaviours[match.group(2)][currentStep] = (ts, float(match.group(3)))
                else:
                    behaviours[match.group(2)] = {currentStep : (ts, float(match.group(3)))}
                continue
            match = thresholdRegex.search(line)
            if match:
                print match.group(0)
                dt = datetime.datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S,%f")
                ts = (dt - datetime.datetime(1970, 1, 1)).total_seconds()
                behaviours["threshold"][currentStep] = (ts, float(match.group(2)))
                continue
            match = stepRegex.search(line)
            if match:
                print match.group(0)
                currentStep = int(match.group(1))
        # now that parsing is done we create suitable output for gnuplot  
        for behaviour in behaviours.keys():
            outfile.write("Time\t" + behaviour + "\t")
        outfile.write("\n")
        for step in reduce(lambda x, y: set(x).union(set(y)), [e.keys() for b,e in behaviours.iteritems()]): # trust me, we are iterating over all steps where at least behaviour delivered data!
            for (behaviour, data) in behaviours.iteritems():
                if step in data:
                    outfile.write(str(step) + "\t" + str(data[step][1]) + "\t")
                else:
                    outfile.write(" \t \t")
            outfile.write("\n")
