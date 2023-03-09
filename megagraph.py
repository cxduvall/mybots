import matplotlib.pyplot as plt
import sys
argv = sys.argv

seed1 = argv[1]
seed2 = argv[2]

pathPID = "fitnessText_" + "PID" + "_" + seed1 + ".txt"
pathNOPID = "fitnessText_" + "NOPID" + "_" + seed2 + ".txt"

with open(pathPID, "r") as f:
    runFitnessStr = f.readlines()[0]
runFitnessStr = runFitnessStr.rstrip("]\n")
runFitnessStr = runFitnessStr.lstrip("[")
runFitnessPID = runFitnessStr.split(", ") # list of lists for each run

with open(pathNOPID, "r") as f:
    runFitnessStr = f.readlines()[0]
runFitnessStr = runFitnessStr.rstrip("]\n")
runFitnessStr = runFitnessStr.lstrip("[")
runFitnessNOPID = runFitnessStr.split(", ") # list of lists for each run

avgPID = []
for i in range(len(runFitnessPID[0])):
    thisAvg = 0
    for j in range(len(runFitnessPID)):
        thisAvg += runFitnessPID[j][i]
    thisAvg /= len(runFitnessPID)
    avgPID.append(thisAvg)

avgNOPID = []
for i in range(len(runFitnessNOPID[0])):
    thisAvg = 0
    for j in range(len(runFitnessNOPID)):
        thisAvg += runFitnessNOPID[j][i]
    thisAvg /= len(runFitnessNOPID)
    avgNOPID.append(thisAvg)

plt.plot(avgNOPID)
plt.plot(avgPID)

# make + save the graph
plt.title("Best Fitness, Averaged Across All Runs")
plt.xlabel("Generation")
plt.ylabel("Fitness")
labels = ["No PID", "PID"]
plt.legend(labels=labels)

plt.savefig("MegaGraph_" + str(seed) + ".png")
