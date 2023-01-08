import constants as c
import numpy as np
import pyrosim.pyrosim as pyrosim
import pybullet as p

class MOTOR:
    def __init__(self, jointName, numFrames):
        self.numFrames = numFrames
        self.jointName = jointName

    def Prepare_To_Act(self):
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.phaseOffset = c.phaseOffset

        self.motorValues = self.amplitude * np.sin(np.linspace(self.phaseOffset, 2*np.pi*self.frequency + self.phaseOffset, self.numFrames))

    def Set_Value(self, val, t, robot):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex = robot,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = self.motorValues[t],
            maxForce = c.maxForce)
