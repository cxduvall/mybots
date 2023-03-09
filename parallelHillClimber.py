from solution import SOLUTION, Cube, Joint, SolutionEncoder
import constants as c
import copy
import os
import random
import json
import sys
import matplotlib.pyplot as plt

class PARALLEL_HILL_CLIMBER:
    def __init__(self, randSeed, runNum, bodyEv, pidNeurons):
        #print("here")
        os.system("rm brain*.nndf")
        #print("rm fitness now...")
        os.system("rm fitness*.txt")
        #print("rm world now...")
        os.system("rm world*.sdf")
        #print("done")
        self.nextAvailableID = 0
        self.parents = {}
        self.randSeed = randSeed
        self.myRandom = random.Random(self.randSeed)
        self.childSeeds = []
        for i in range(c.populationSize):
            childSeed = self.myRandom.randrange(sys.maxsize)
            self.parents[i] = SOLUTION(self.nextAvailableID, childSeed, bodyEv, pidNeurons)
            self.nextAvailableID += 1
            self.childSeeds.append(childSeed)
        #self.showFirst = showFirst
        self.runNum = runNum
        self.populationSize = c.populationSize
        self.pidStr = "PID" if pidNeurons else "NOPID"

    def Evolve(self, showFirst=False):
        self.evaluations = 0
        #firstDirect = not self.showFirst

        self.Evaluate("create", self.parents, direct=(not showFirst), firstGen=True)
        self.initBestFitness()

        with open("firstGen_run" + str(self.runNum) + "_" + self.pidStr + "_seed" + str(self.randSeed)
            + "_pop" + str(c.populationSize)
            + "_gens" + str(c.numberOfGenerations)
            + "_evalSecs" + str(c.numSecs)
            + ".json", "w") as f:
            #print(type(self.parents[0]))
            #print(self.parents)
            jsonStr = json.dumps(self.parents, cls=SolutionEncoder)
            f.write(jsonStr)

        # TODO shows first gen -- can rm for non-video purposes
        #for parent in self.parents.values():
        #    parent.Evaluate("view", direct=True)
        
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation(currentGeneration + 1)

        # store all final-gen creatures for this run in a file named lastGen_seedX_popX_gensX_evalSecsX
        with open("lastGen_run" + str(self.runNum) + "_" + self.pidStr + "_seed" + str(self.randSeed)
            + "_pop" + str(c.populationSize)
            + "_gens" + str(c.numberOfGenerations)
            + "_evalSecs" + str(c.numSecs)
            + ".json", "w") as f:
            print(type(self.parents[0]))
            jsonStr = json.dumps(self.parents, cls=SolutionEncoder)
            f.write(jsonStr)

        for fitness in self.lineFitness:
            plt.plot(fitness)
        plt.title("Negative Distance from Origin (Smaller AbsVal is Better) - By Lineage")
        plt.xlabel("Generation")
        plt.ylabel("Fitness")
        labels = []
        for i in range(len(self.lineFitness)):
            labels.append("Lineage " + str(i) + ": " + str(self.childSeeds[i]))
        plt.legend(labels=labels)
        plt.savefig("lineageGraph_run" + str(self.runNum) + "_" + self.pidStr + "_" + str(self.randSeed) + ".png")
        plt.clf() # clears the figure

        # save lineage graph as txt
        with open("lineageText_run" + str(self.runNum) + "_" + self.pidStr + "_" + str(self.randSeed) + ".txt", "w") as f:
            f.write(str(self.lineFitness))


        totalMuts = 0
        for parent in self.parents.values():
            totalMuts += parent.actualBodyMutations
        avgMuts = totalMuts / len(self.parents)

        return [self.bestFitness, avgMuts]

    def Evolve_For_One_Generation(self, generation, direct=True):
        self.Spawn()
        self.Evaluate("mutate", self.children, direct=direct)
        self.Print()
        self.Select(generation)

    def Evaluate(self, command, solutions, firstGen=False, direct=True):
        print("\nRUN", self.runNum + ", GEN:", self.evaluations)
        #print("direct:", direct, "\n")
        self.evaluations += 1
        
        for i in range(c.populationSize):
            solutions[i].Start_Simulation(command, direct=direct) # handles mutation
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

        print("SELF.PARENTS.VALUES():", list(self.parents.values()))
        print("SELF.PARENTS run seeds:", list(map(lambda x: x.randSeed, self.parents.values())))

        for parent in self.parents.values():
            parent.Evaluate("view", direct=False)

        #self.worstFitnessExample.Evaluate(False)
        #self.parents[bestKey].Evaluate(False)

    def Print(self):
        pass
        '''
        print('\n')
        for key in self.parents.keys():
            parent = self.parents[key]
            child = self.children[key]
            print(parent.Get_Fitness(), child.Get_Fitness())
        print('\n')
        '''

    def Spawn(self):
        self.children = {}
        for key in self.parents.keys():
            self.children[key] = copy.deepcopy(self.parents[key])
            self.children[key].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1
            self.children[key].seed_random(self.myRandom.randrange(sys.maxsize))

    def initBestFitness(self):
        self.lineFitness = []
        self.bestFitness = []
        bestFitness = None
        parents = list(self.parents.values())
        for i in range(len(parents)):
            parent = parents[i]
            parentFitness = parent.Get_Fitness()
            self.lineFitness.append([-1 * parentFitness])
            if bestFitness is None or parentFitness < bestFitness:
                bestFitness = parentFitness
        self.bestFitness.append(-1 * bestFitness)

    def Select(self, generation):
        bestFitness = None
        bestFitnessExample = None
        keys = list(self.parents.keys())
        for i in range(len(keys)):
            #print("\nKEY: "+str(key))
            #print("PARENTS: " + str(self.parents))
            #print("CHILDREN: " + str(self.children))
            key = keys[i]
            parent = self.parents[key]
            child = self.children[key]
            parentFitness = parent.Get_Fitness()
            childFitness = child.Get_Fitness()
            self.lineFitness[i].append(-1 * min(parentFitness,
                childFitness))
            if childFitness < parentFitness:
                self.parents[key] = child
                #print("\nREPLACING FITNESS", parentFitness, "WITH", childFitness)
            if bestFitness is None or parentFitness < bestFitness:
                bestFitness = parentFitness
                bestFitnessExample = parent
            if childFitness < bestFitness:
                bestFitness = childFitness
                bestFitnessExample = child
        self.bestFitness.append(-1 * bestFitness)
