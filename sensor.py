import pyrosim.pyrosim as pyrosim
import numpy as np

class SENSOR:
    def __init__(self, linkName, numFrames):
        self.linkName = linkName
        self.numFrames = numFrames
        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        self.values = np.zeros(self.numFrames)

    def Get_Value(self, t):
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        if(t == self.numFrames - 1):
            print(self.values)
