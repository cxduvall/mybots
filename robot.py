import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os

class ROBOT:
    def __init__(self, numFrames, id):
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense(numFrames)
        self.Prepare_To_Act(numFrames)
        self.nn = NEURAL_NETWORK("brain" + str(id) + ".nndf")
        os.system("rm brain" + str(id) + ".nndf")
        self.myID = id

    def Prepare_To_Sense(self, numFrames):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName, numFrames)

    def Prepare_To_Act(self, numFrames):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName, numFrames)

    def Sense(self, t):
        for sensor in self.sensors.values():
            sensor.Get_Value(t)

    def Act(self, t):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
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
        stateOfLinkZero = p.getLinkState(self.robotId, 0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        with open("tmp" + str(self.myID) + ".txt", "w") as f:
            f.write(str(xCoordinateOfLinkZero))
        os.system("mv tmp" + str(self.myID) + ".txt fitness" + str(self.myID) + ".txt")
