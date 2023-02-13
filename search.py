import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
import numpy as np
import random
import sys
import matplotlib.pyplot as plt
import constants as c

runFitness = []
numRuns = c.numRuns
seeds = []
for i in range(numRuns):
    print("\n--- DOING SIM NUMBER:", i, "---\n")
    randSeed = random.randrange(sys.maxsize)
    seeds.append(randSeed)
    showFirst = (i == numRuns - 1)
    phc = PARALLEL_HILL_CLIMBER(randSeed, showFirst)
    runFitness.append(phc.Evolve())

print("\n\n\n\n\n--- SHOWING BEST!! ---")
phc.Show_Best()

fitArr = np.asarray(runFitness)
print("fitArr.shape:", fitArr.shape)
print("\nrunFitness:", runFitness)

# save fitness curves to a UNIQUELY named file (random name based on timestamp)
for i in range(numRuns):
    plt.plot(runFitness[i])
plt.title("Horizontal Distance from Origin (Larger is Better)")
plt.xlabel("Generation")
plt.ylabel("Fitness")
labels = []
for i in range(numRuns):
    labels.append("Run " + str(i) + ": " + str(seeds[i]))
plt.legend(labels=labels)
plt.show()
plt.savefig("fitnessGraph_" + str(seeds[0]) + ".png")
