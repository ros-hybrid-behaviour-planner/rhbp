#! /usr/bin/python
import re
import datetime
import sys

if len(sys.argv) < 2:
    print "usage: %s <logfile>" % sys.argv[0]
    sys.exit()
logfile = sys.argv[1]

activationRegex = re.compile(r'(\d+-\d+-\d+\s\d+:\d+:\d+,\d+):\s+activation\s+of\s+([a-zA-Z]+)\s+after\s+this\s+step:\s+(\d+\.\d+)')
stepRegex = re.compile(r'###################################### STEP (\d+) ######################################')

behaviours = {
    "startBehaviour" : None,
    "goHomeBehaviour" : None,
    "landBehaviour" : None,
    "moveBehaviour" : None,
    "collisionAvoidence" : None,
    "movementArbiter" : None
}

with open("activationFlow.dat", 'w') as outfile:
    for (behaviour, data) in behaviours.iteritems():
        outfile.write("Time\t" + behaviour + "\t")
    outfile.write("\n")
    with open(logfile, 'r') as infile:
        for line in infile.readlines():
            match = activationRegex.search(line)
            if match:
                dt = datetime.datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S,%f")
                ts = (dt - datetime.datetime(1970, 1, 1)).total_seconds()
                behaviours[match.group(2)] = (ts, float(match.group(3)))
                continue
            match = stepRegex.search(line)
            if match:
                try:
                    for (behaviour, data) in behaviours.iteritems():
                        outfile.write(match.group(1) + "\t" + str(data[1]) + "\t")
                    outfile.write("\n")
                except:
                    pass
            # "outfile.print("Time	goHomeBehaviour	Time	landBehaviour	Time	MoveBehaviour	Time	selectTargetBehaviour	Time	startBehaviour	Time	activationThreshold\n")
