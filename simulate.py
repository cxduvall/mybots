from simulation import SIMULATION
import sys

#print("in simulate!")
numSecs = sys.argv[1]
directOrGUI = sys.argv[2]
solutionID = sys.argv[3]
pid = sys.argv[4]
simulation = SIMULATION(directOrGUI, solutionID, int(numSecs), bool(pid))
simulation.Run()
simulation.Get_Fitness()
