import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR

class ROBOT:
    def __init__(self, numFrames):
        self.motors = {}
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense(numFrames)

    def Prepare_To_Sense(self, numFrames):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName, numFrames)

    def Sense(self, t):
        for sensor in self.sensors.values():
            sensor.Get_Value(t)
