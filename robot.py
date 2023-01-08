import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR

class ROBOT:
    def __init__(self, numFrames):
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense(numFrames)
        self.Prepare_To_Act(numFrames)

    def Prepare_To_Sense(self, numFrames):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName, numFrames)

    def Prepare_To_Act(self, numFrames):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName, numFrames)
            self.motors[jointName].Prepare_To_Act()

    def Sense(self, t):
        for sensor in self.sensors.values():
            sensor.Get_Value(t)

    def Act(self, t):
         for motor in self.motors.values():
             motor.Set_Value(t, self.robotId)

    def Save_Sensor_Values(self):
        for sensor in self.sensors.values:
            sensor.Save_Values()

    def Save_Motor_Values(self):
        for motor in self.motors.values:
            motor.Save_Values()
