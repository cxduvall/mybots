from solution import SOLUTION
import constants as c
import copy

class HILL_CLIMBER:
    def __init__(self):
        self.parent = SOLUTION()

    def Evolve(self):
        self.parent.Evaluate()
        for currentGeneration in range(c.numberOfGenerations):
            if currentGeneration == 0:
                self.Evolve_For_One_Generation(False)
            else:
                self.Evolve_For_One_Generation()
    
    def Evolve_For_One_Generation(self, direct=True):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate(direct)
        self.Select()
        self.Print()

    def Show_Best(self):
        self.parent.Evaluate(False)

    def Print(self):
        print(self.parent.Evaluate(), self.child.Evaluate())

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.Mutate()

    def Select(self):
        if self.parent.Evaluate() > self.child.Evaluate():
            self.parent = self.child
