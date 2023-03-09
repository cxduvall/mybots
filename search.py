import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
import numpy as np
import random
import sys
import matplotlib.pyplot as plt
import constants as c

argv = sys.argv

runFitness = []
numRuns = c.numRuns
seeds = []
avgMuts = []

# rn, shows first creatures, and last creatures, on the last run.

# command must have BODYEV or NOBODYEV, PID or NOPID, then optionally (a) seed(s)
bodyEv = (argv[1] == "BODYEV")
pidStr = argv[2]
pidNeurons = (pidStr == "PID")

# if give (a) seed(s), just do 1 run w each
if len(argv) > 3:
    for i in range(len(argv) - 3):
        print("\n--- DOING RUN NUMBER:", i, "---\n")
        randSeed = int(sys.argv[i + 2])
        print("Whole-run randSeed:", randSeed)
        seeds.append(randSeed)
        phc = PARALLEL_HILL_CLIMBER(randSeed, str(i), bodyEv, pidNeurons)
        result = phc.Evolve(False)
        runFitness.append(result[0])
        avgMuts.append(result[1])
else: # no seeds specified
    for i in range(numRuns):
        print("\n--- DOING SIM NUMBER:", i, "---\n")
        randSeed = random.randrange(sys.maxsize)
        print("Whole-run randSeed:", randSeed)
        seeds.append(randSeed)
        #showFirst = (i == numRuns - 1)
        phc = PARALLEL_HILL_CLIMBER(randSeed, str(i), bodyEv, pidNeurons)
        result = phc.Evolve(False)
        runFitness.append(result[0])
        avgMuts.append(result[1])

print("\n\n\n\n\n--- SHOWING BEST!! ---")
#phc.Show_Best()

fitArr = np.asarray(runFitness)
print("fitArr.shape:", fitArr.shape)
print("\nrunFitness:", runFitness)
print("\navgMuts:", avgMuts)

# save fitness curves to a UNIQUELY named file (random name based on timestamp)
for i in range(len(runFitness)):
#for genIndex in range(len(runFitness[0])):
    #maxForGen = runFitness[0][genIndex]
    #for runIndex in range(len(runFitness)):
    #    if runFitness[runIndex][genIndex] > maxForGen:
    #        maxForGen = runFitness[runIndex][genIndex]
    plt.plot(runFitness[i])

plt.title("Negative Distance from Origin (Smaller AbsVal is Better)")
plt.xlabel("Generation")
plt.ylabel("Fitness")
labels = []
for i in range(len(runFitness)):
    labels.append("Run " + str(i) + ": " + str(seeds[i]))
plt.legend(labels=labels)

plt.savefig("fitnessGraph_" + pidStr + "_" + str(seeds[0]) + ".png")
print("ALL SEEDS FOR THIS SET OF RUNS:", seeds)

with(open("fitnessText_" + pidStr + "_" + str(seeds[0]) + ".txt", "w")) as f:
    f.write(str(runFitness))

#plt.show() # clears the figure # TODO seg fault -- why?
