from solution import SOLUTION
import constants as c
import copy
import os

class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        #print("here")
        os.system("rm brain*.nndf")
        #print("rm fitness now...")
        os.system("rm fitness*.txt")
        #print("rm world now...")
        os.system("rm world*.sdf")
        #print("done")
        self.nextAvailableID = 0
        self.parents = {}
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Evolve(self):
        self.Evaluate(self.parents)
            
        for currentGeneration in range(c.numberOfGenerations):
            if currentGeneration == 0:
                self.Evolve_For_One_Generation(currentGeneration + 1, False)
            else:
                self.Evolve_For_One_Generation(currentGeneration + 1)

    def Evolve_For_One_Generation(self, generation, direct=True):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Select(generation)
        self.Print()

    def Evaluate(self, solutions):
        for i in range(c.populationSize):
            solutions[i].Start_Simulation()
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
        self.worstFitnessExample.Evaluate(False)
        self.parents[bestKey].Evaluate(False)

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

    def Mutate(self):
        for child in self.children.values():
            child.Mutate()

    def Select(self, generation):
        worstFitness = None
        worstFitnessExample = None
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
            if worstFitness is None or parentFitness > worstFitness:
                worstFitness = parentFitness
                worstFitnessExample = parent
            if childFitness > worstFitness:
                worstFitness = childFitness
                worstFitnessExample = child 
        if generation == 1:
            self.worstFitnessExample = worstFitnessExample
