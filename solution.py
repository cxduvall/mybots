import numpy as np
import os
import pyrosim.pyrosim as pyrosim
import random
import time
import constants as c
import random

class SOLUTION:
    def __init__(self, id):
        self.myID = id

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
        #self.Create_Platform()

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
        randomRow = random.randint(0, self.numSensorNeurons - 1)
        randomColumn = random.randint(0, self.numMotorNeurons - 1)
        self.weights[randomRow,randomColumn] = random.random() * 2 - 1

    def Create_World(self):
        pyrosim.Start_SDF("world" + str(self.myID) + ".sdf")
        #pyrosim.Send_Cube(name="Box", pos=[self.x-2,self.y+2,self.z] , size=[self.width*self.scale,self.length*self.scale,self.height*self.scale])
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body" + str(self.myID) + ".urdf")

        numLinks = random.randint(2,6)
        numJoints = numLinks - 1
        firstSensorIndex = random.randint(0, numLinks - 1)
        sensors = [False] * numLinks
        linkSizes = [None] * numLinks
        sensors[firstSensorIndex] = True
        totalLength = 0

        for i in range(numLinks):
            if random.uniform(0,1) > 0.5:
                sensors[i] = True
            l = random.uniform(0.5, 6)
            w = random.uniform(0.5, 4)
            h = random.uniform(0.5, 4)
            linkSizes[i] = [l,w,h]
            totalLength += l

        finalH = 2.5

        for j in range(numLinks):

            # current link
            xPos = linkSizes[j][0]/2 if j != 0 else ((-1 * (totalLength / 2)) + linkSizes[j][0]/2)
            zPos = 0 if j != 0 else finalH

            name = "Link"+str(j)
            #yOffset =
            #zOffset =
            pos = [xPos,0,zPos]
            if sensors[j]:
                pyrosim.Send_Cube(name=name, pos=pos, size=linkSizes[j],
                    matName="Green", matRGBA=[0.0,1.0,0.0,1.0])
            else:
                pyrosim.Send_Cube(name=name, pos=pos, size=linkSizes[j],
                    matName="Blue", matRGBA=[0.0,0.5,1.0,1.0])

            # add a joint if not on the last link
            if j != numLinks - 1:
                pyrosim.Send_Joint(name = "Link"+str(j)+"_Link"+str(j+1) , parent= "Link"+str(j), child = "Link"+str(j+1) , type = "revolute",
                    position = [xPos + linkSizes[j][0]/2,0,zPos], jointAxis="0 0 1")
        
        self.numLinks = numLinks
        self.sensors = sensors

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

        j = 0
        numSensorNeurons = 0
        for i in range(self.numLinks):
            if self.sensors[i]:
                pyrosim.Send_Sensor_Neuron(name = j, linkName = "Link"+str(i))
                j += 1
                numSensorNeurons += 1

        for i in range(self.numLinks - 1):
            pyrosim.Send_Motor_Neuron( name = j, jointName = "Link"+str(i)+"_Link"+str(i+1))
            j += 1

        numMotorNeurons = self.numLinks - 1

        self.numSensorNeurons = numSensorNeurons
        self.numMotorNeurons = numMotorNeurons
        self.weights = 2*np.random.rand(self.numSensorNeurons,self.numMotorNeurons)
        self.weights -= 1

        for currentRow in range(numSensorNeurons): # sensor
            for currentColumn in range(numMotorNeurons): # motor
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+numSensorNeurons, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()
