from solution import SOLUTION
import constants as c
import copy
import os
import random

class PARALLEL_HILL_CLIMBER:
    def __init__(self, randSeed, showFirst):
        #print("here")
        os.system("rm brain*.nndf")
        #print("rm fitness now...")
        os.system("rm fitness*.txt")
        #print("rm world now...")
        os.system("rm world*.sdf")
        #print("done")
        self.nextAvailableID = 0
        self.parents = {}
        self.myRandom = random.Random(randSeed)
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID, self.myRandom)
            self.nextAvailableID += 1
        self.showFirst = showFirst

    def Evolve(self):
        self.evaluations = 0
        firstDirect = not self.showFirst
        self.Evaluate(self.parents, direct=firstDirect, firstGen=True)
        self.initBestFitness(firstDirect)
        
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation(currentGeneration + 1)

        return self.bestFitness

    def Evolve_For_One_Generation(self, generation, direct=True):
        self.Spawn()
        self.Evaluate(self.children, direct=direct)
        self.Select(generation)
        self.Print()

    def Evaluate(self, solutions, firstGen=False, direct=True):
        print("\nEVALUATING:", self.evaluations)
        print("direct:", direct, "\n")
        self.evaluations += 1
        
        for i in range(c.populationSize):
            solutions[i].Start_Simulation(direct=direct, firstGen=firstGen) # handles mutation
        for i in range(c.populationSize):
            solutions[i].Wait_For_Simulation_To_End()

    def Show_Best(self):
        best = None
        bestKey = None
        for key in self.parents.keys():
            parent = self.parents[key]
            fitness = parent.Get_Fitness()
            if best is None or fitness < best:
                best = fitness
                bestKey = key
        #("BEST KEY:", bestKey, best)

        for parent in self.parents.values():
            parent.Evaluate(direct=False)

        #self.worstFitnessExample.Evaluate(False)
        #self.parents[bestKey].Evaluate(False)

    def Print(self):
        print('\n')
        for key in self.parents.keys():
            parent = self.parents[key]
            child = self.children[key]
            print(parent.Get_Fitness(), child.Get_Fitness())
        print('\n')

    def Spawn(self):
        self.children = {}
        for key in self.parents.keys():
            self.children[key] = copy.deepcopy(self.parents[key])
            self.children[key].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def initBestFitness(self, firstDirect):
        self.bestFitness = []
        bestFitness = None
        for key in self.parents.keys():
            parent = self.parents[key]
            parentFitness = parent.Get_Fitness()
            if bestFitness is None or parentFitness < bestFitness:
                bestFitness = parentFitness
        self.bestFitness.append(-1 * bestFitness)

    def Select(self, generation):
        bestFitness = None
        bestFitnessExample = None
        for key in self.parents.keys():
            #print("\nKEY: "+str(key))
            #print("PARENTS: " + str(self.parents))
            #print("CHILDREN: " + str(self.children))
            parent = self.parents[key]
            child = self.children[key]
            parentFitness = parent.Get_Fitness()
            childFitness = child.Get_Fitness()
            if parentFitness > childFitness:
                self.parents[key] = child
            if bestFitness is None or parentFitness < bestFitness:
                bestFitness = parentFitness
                bestFitnessExample = parent
            if childFitness < bestFitness:
                bestFitness = childFitness
                bestFitnessExample = child
        self.bestFitness.append(-1 * bestFitness)
