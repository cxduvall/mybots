import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os
import constants as c
import math

class ROBOT:
    def __init__(self, numFrames, id):
        print("in robot!")
        self.robotId = p.loadURDF("body" + str(id) + ".urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense(numFrames)
        self.Prepare_To_Act(numFrames)
        self.nn = NEURAL_NETWORK("brain" + str(id) + ".nndf")
        os.system("rm brain" + str(id) + ".nndf")
        os.system("rm body" + str(id) + ".urdf")
        self.myID = id
        print("done w robot!")

    def Prepare_To_Sense(self, numFrames):
        self.sensors = {}
        for linkName in pyrosim.LinkNamesToIndices():
            if linkName not in ["Pillar", "Table", "Base"]:
                self.sensors[linkName] = SENSOR(linkName, numFrames)

    def Prepare_To_Act(self, numFrames):
        self.motors = {}
        for jointName in pyrosim.JointNamesToIndices():
            if jointName not in ["Pillar_Table", "Base_Pillar"]:
                self.motors[jointName] = MOTOR(jointName, numFrames)

    def Prepare(self):
        pyrosim.Prepare_To_Simulate(self.robotId)

    def Sense(self, t):
        for key in self.sensors.keys():
            sensor = self.sensors[key]
            #print("sensing w sensor", key)
            sensor.Get_Value(t)
            #print("sensed successfully.")

    def Act(self, t):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = c.motorJointRange * self.nn.Get_Value_Of(neuronName)
                #print("desired angle for joint " + jointName + ": ", desiredAngle)
                self.motors[jointName].Set_Value(desiredAngle, self.robotId)

    def Think(self, t):
        self.nn.Update()
        #self.nn.Print()

    def Save_Sensor_Values(self):
        for sensor in self.sensors.values:
            sensor.Save_Values()

    def Save_Motor_Values(self):
        for motor in self.motors.values:
            motor.Save_Values()

    def Get_Fitness(self):
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        xPos = basePosition[0]
        yPos = basePosition[1]
        zPos = basePosition[2]
        distance = math.sqrt((xPos**2) + (yPos**2))
        fitness = zPos * -1
        with open("tmp" + str(self.myID) + ".txt", "w") as f:
            f.write(str(fitness))
        os.system("mv tmp" + str(self.myID) + ".txt fitness" + str(self.myID) + ".txt")
