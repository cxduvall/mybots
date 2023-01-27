import numpy as np
import os
import pyrosim.pyrosim as pyrosim
import random
import time
import constants as c

class SOLUTION:
    def __init__(self, id):
        self.myID = id
        self.weights = 2*np.random.rand(c.numSensorNeurons,c.numMotorNeurons)
        self.weights -= 1

        self.baseHeight = 0.5
        self.platformHeight = 0.5
        self.platformX = 0
        self.platformY = 0
        self.pillarHeight = 1
        self.platformWidth = 4
        self.platformLength = 4

        self.length = 1
        self.width = 1
        self.height = 1
        self.x = self.platformX
        self.y = self.platformY
        self.z = 0.25 #1.25 + self.baseHeight + self.pillarHeight + self.platformHeight
        self.scale = 1

    def Start_Simulation(self, direct=True):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        self.Create_Platform()

        print("about to syscall simulate.py")

        if direct:
            command = "python3 simulate.py DIRECT " + str(self.myID) + " 2" # 2&>1 &
        else:
            command = "python3 simulate.py GUI " + str(self.myID) + " 2" # 2&>1 &
        print("Command:", command)
        os.system(command)

    def Wait_For_Simulation_To_End(self, direct=True):
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        
        with open(fitnessFileName, "r") as f:
            self.fitness = float(f.readline().strip())
        os.system("rm fitness" + str(self.myID) + ".txt")
        os.system("rm world" + str(self.myID) + ".sdf")
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
        randomRow = random.randint(0, c.numSensorNeurons - 1)
        randomColumn = random.randint(0, c.numMotorNeurons - 1)
        self.weights[randomRow,randomColumn] = random.random() * 2 - 1

    def Create_World(self):
        pyrosim.Start_SDF("world" + str(self.myID) + ".sdf")
        #pyrosim.Send_Cube(name="Box", pos=[self.x-2,self.y+2,self.z] , size=[self.width*self.scale,self.length*self.scale,self.height*self.scale])
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body" + str(self.myID) + ".urdf")
        
        pyrosim.Send_Cube(name="Torso", pos=[self.x,self.y,self.z+1] , size=[self.width*self.scale,self.length*self.scale, self.height*self.scale])
        pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [self.x-0.4,self.y-0.5,self.z+1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="BackLeg", pos=[0,-0.5,0] , size=[0.2,1,0.2])
        pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [self.x-0.4,self.y+0.5,self.z+1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5,0] , size=[0.2,1,0.2])
        pyrosim.Send_Joint(name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [self.x+0.4,self.y-0.5,self.z+1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[0,-0.5,0] , size=[0.2,1,0.2])
        pyrosim.Send_Joint(name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [self.x+0.4,self.y+0.5,self.z+1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[0,0.5,0] , size=[0.2,1,0.2])

        pyrosim.Send_Joint(name = "FrontLeg_FrontLowerLeg" , parent= "FrontLeg" , child = "FrontLowerLeg" , type = "revolute", position = [0,1,0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0,0,-0.5] , size=[0.2,0.2,1])
        pyrosim.Send_Joint(name = "BackLeg_BackLowerLeg" , parent= "BackLeg" , child = "BackLowerLeg" , type = "revolute", position = [0,-1,0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLowerLeg", pos=[0,0,-0.5] , size=[0.2,0.2,1])
        pyrosim.Send_Joint(name = "LeftLeg_LeftLowerLeg" , parent= "LeftLeg" , child = "LeftLowerLeg" , type = "revolute", position = [0,-1,0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0,0,-0.5] , size=[0.2,0.2,1])
        pyrosim.Send_Joint(name = "RightLeg_RightLowerLeg" , parent= "RightLeg" , child = "RightLowerLeg" , type = "revolute", position = [0,1,0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="RightLowerLeg", pos=[0,0,-0.5] , size=[0.2,0.2,1])

        #self.Make_Platform()

        pyrosim.End()

    def Make_Platform(self):

        pyrosim.Send_Cube(name="Base", pos=[self.platformX,self.platformY, self.baseHeight/2], size=[self.platformLength*1.5, self.platformWidth*1.5, 0.5],
            matName="Red", matRGBA=[1.0,0.5,0.5,1.0])
        pyrosim.Send_Joint(name="Base_Pillar", parent="Base", child="Pillar", type = "revolute", position=[self.platformX,self.platformY, self.baseHeight])
        pyrosim.Send_Cube(name="Pillar", pos=[0, 0, self.pillarHeight/2], size=[0.5,0.5,self.pillarHeight],
            matName="Red", matRGBA=[1.0,0.5,0.5,1.0])
        pyrosim.Send_Joint(name = "Pillar_Table" , parent= "Pillar" , child = "Table" , type = "revolute", position = [self.platformX,self.platformY,self.pillarHeight], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="Table", pos=[0,0, self.platformHeight/2] , size=[self.platformLength,self.platformWidth,self.platformHeight],
            matName="Red", matRGBA=[1.0,0.5,0.5,1.0])

    def Create_Platform(self):
        pyrosim.Start_URDF("platform" + str(self.myID) + ".urdf")
        self.Make_Platform()
        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "FrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "LeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = "RightLeg")
        pyrosim.Send_Sensor_Neuron(name = 4, linkName = "FrontLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 5, linkName = "BackLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 6, linkName = "LeftLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 7, linkName = "RightLowerLeg")

        pyrosim.Send_Motor_Neuron( name = 8, jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 9, jointName = "Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 10, jointName = "Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron( name = 11, jointName = "Torso_RightLeg")
        pyrosim.Send_Motor_Neuron( name = 12, jointName = "FrontLeg_FrontLowerLeg")
        pyrosim.Send_Motor_Neuron( name = 13, jointName = "BackLeg_BackLowerLeg")
        pyrosim.Send_Motor_Neuron( name = 14, jointName = "LeftLeg_LeftLowerLeg")
        pyrosim.Send_Motor_Neuron( name = 15, jointName = "RightLeg_RightLowerLeg")

        for currentRow in range(c.numSensorNeurons): # sensor
            for currentColumn in range(c.numMotorNeurons): # motor
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+c.numSensorNeurons, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()