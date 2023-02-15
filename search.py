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

# rn, shows first creatures, and last creatures, on the last run.

if len(argv) > 1: # if give a seed, just do 1 run w it
    for i in range(len(argv) - 1):
        print("\n--- DOING SIM NUMBER:", i, "---\n")
        randSeed = int(sys.argv[i + 1])
        print("Whole-run randSeed:", randSeed)
        seeds.append(randSeed)
        phc = PARALLEL_HILL_CLIMBER(randSeed, True, str(i))
        runFitness.append(phc.Evolve())

else:
    for i in range(numRuns):
        print("\n--- DOING SIM NUMBER:", i, "---\n")
        randSeed = random.randrange(sys.maxsize)
        print("Whole-run randSeed:", randSeed)
        seeds.append(randSeed)
        showFirst = (i == numRuns - 1)
        phc = PARALLEL_HILL_CLIMBER(randSeed, showFirst, str(i))
        runFitness.append(phc.Evolve())

print("\n\n\n\n\n--- SHOWING BEST!! ---")
phc.Show_Best()

fitArr = np.asarray(runFitness)
print("fitArr.shape:", fitArr.shape)
print("\nrunFitness:", runFitness)

# save fitness curves to a UNIQUELY named file (random name based on timestamp)
for i in range(len(runFitness)):
    plt.plot(runFitness[i])
plt.title("Horizontal Distance from Origin (Larger is Better)")
plt.xlabel("Generation")
plt.ylabel("Fitness")
labels = []
for i in range(len(runFitness)):
    labels.append("Run " + str(i) + ": " + str(seeds[i]))
plt.legend(labels=labels)
plt.savefig("fitnessGraph_" + str(seeds[0]) + ".png")
plt.show() # clears the figure

print("ALL SEEDS FOR THIS SET OF RUNS:", seeds)
