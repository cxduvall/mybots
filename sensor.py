import pyrosim.pyrosim as pyrosim
import numpy as np

class SENSOR:
    def __init__(self, linkName, numFrames):
        self.linkName = linkName
        self.values = np.zeros(numFrames)
        self.numFrames = numFrames

    def Get_Value(self, t):
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        if(t == self.numFrames - 1):
            print(self.values)
