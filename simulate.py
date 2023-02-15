from simulation import SIMULATION
import sys

#print("in simulate!")
numSecs = sys.argv[1]
directOrGUI = sys.argv[2]
solutionID = sys.argv[3]
simulation = SIMULATION(directOrGUI, solutionID, int(numSecs))
simulation.Run()
simulation.Get_Fitness()
