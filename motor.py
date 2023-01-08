import constants as c
import numpy as np

class MOTOR:
    def __init__(self, jointName, numFrames):
        self.numFrames = numFrames
        self.Prepare_To_Act(jointName)

    def Prepare_To_Act(self, jointName):
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.phaseOffset = c.phaseOffset

        self.motorValues = self.amplitude * np.sin(np.linspace(self.phaseOffset, 2*np.pi*self.frequency + self.phaseOffset, self.numFrames))

    def Set_Value(self, val, t):
        pass
        
        #pyrosim.Set_Motor_For_Joint(
            #bodyIndex = robotId,
            #jointName = "Torso_BackLeg",
            #controlMode = p.POSITION_CONTROL,
            #targetPosition = backTargetAngles[i],
            #maxForce = c.maxForce)

        #pyrosim.Set_Motor_For_Joint(
            #bodyIndex = robotId,
            #jointName = "Torso_FrontLeg",
            #controlMode = p.POSITION_CONTROL,
            #targetPosition = frontTargetAngles[i],
            #maxForce = c.maxForce)
