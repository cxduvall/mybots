import numpy as np
import os
import pyrosim.pyrosim as pyrosim
import random
import time

class SOLUTION:
    def __init__(self, id):
        self.myID = id
        self.weights = 2*np.random.rand(3,2)
        self.weights -= 1

        self.length = 1
        self.width = 1
        self.height = 1
        self.x = 0
        self.y = 0
        self.z = 1.5
        self.scale = 1

    def Start_Simulation(self, direct=True):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        if direct:
            os.system("python3 simulate.py DIRECT " + str(self.myID) + " &")
        else:
            os.system("python3 simulate.py GUI " + str(self.myID) + " &")

    def Wait_For_Simulation_To_End(self, direct=True):
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        
        with open(fitnessFileName, "r") as f:
            self.fitness = float(f.readline().strip())
        os.system("rm fitness" + str(self.myID) + ".txt")
        print(self.fitness)
        return self.fitness

    def Get_Fitness(self):
        if self.fitness is not None:
            return self.fitness

    def Evaluate(self, direct=True):
        self.Start_Simulation(direct)
        self.Wait_For_Simulation_To_End(direct)

    def Set_ID(self, id):
        self.myID = id

    def Mutate(self):
        randomRow = random.randint(0, 2)
        randomColumn = random.randint(0, 1)
        self.weights[randomRow,randomColumn] = random.random() * 2 - 1

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[self.x-2,self.y+2,self.z] , size=[self.width*self.scale,self.length*self.scale,self.height*self.scale])
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")
        
        pyrosim.Send_Cube(name="Torso", pos=[self.x,self.y,self.z] , size=[self.width*self.scale,self.length*self.scale, self.height*self.scale])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [self.x-0.5,self.y,self.z-0.5])
        pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[self.width*self.scale,self.length*self.scale,self.height*self.scale])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [self.x+0.5,self.y,self.z-0.5])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[self.width*self.scale, self.length*self.scale, self.height*self.scale])

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4, jointName = "Torso_FrontLeg")

        for currentRow in range(3): # sensor
            for currentColumn in range(2): # motor
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+3, weight = self.weights[currentRow][currentColumn])

        #pyrosim.Send_Synapse( sourceNeuronName = 1 , targetNeuronName = 3 , weight = 1.0 )
        #pyrosim.Send_Synapse( sourceNeuronName = 2 , targetNeuronName = 3 , weight = -1.0 )
        #pyrosim.Send_Synapse( sourceNeuronName = 1 , targetNeuronName = 4 , weight = -1.0 )
        #pyrosim.Send_Synapse( sourceNeuronName = 2 , targetNeuronName = 4 , weight = -1.0 )

        pyrosim.End()