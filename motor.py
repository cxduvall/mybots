import constants as c
import numpy as np
import pyrosim.pyrosim as pyrosim
import pybullet as p

class MOTOR:
    def __init__(self, jointName, numFrames):
        self.numFrames = numFrames
        self.jointName = jointName

    def Set_Value(self, angle, robot):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex = robot,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = angle,
            maxForce = c.maxForce)

    def Set_Speed(self, speed, robot, maxForce):
        pyrosim.Set_Motor_Velocity_For_Joint(
            bodyIndex = robot,
            jointName = self.jointName,
            controlMode = p.VELOCITY_CONTROL,
            targetVelocity = speed,
            maxForce = maxForce)
            