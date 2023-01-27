import pyrosim.pyrosim as pyrosim
import numpy as np

class SENSOR:
    def __init__(self, linkName, numFrames):
        self.linkName = linkName
        self.numFrames = numFrames
        self.Prepare_To_Sense()

    def Prepare_To_Sense(self):
        self.values = np.zeros(self.numFrames)

    def Get_Value(self, t):
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        #print("in sensor get_value; self.values[t[]:", self.values[t])

    def Save_Values(self):
        np.save("data/" + self.linkName + "SensorValues.npy", self.values)
