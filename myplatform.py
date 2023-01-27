import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os
import constants as c
import math

class PLATFORM:
    def __init__(self, numFrames, id, fps):
        self.fps = fps
        #print("in platform!")
        self.platformId = p.loadURDF("platform" + str(id) + ".urdf")
        pyrosim.Prepare_To_Simulate(self.platformId)
        self.Prepare_To_Act(numFrames)
        os.system("rm platform" + str(id) + ".urdf")
        self.myID = id

    def Prepare(self):
        pyrosim.Prepare_To_Simulate(self.platformId)

    def Prepare_To_Act(self, numFrames):
        self.motor = MOTOR("Pillar_Table", numFrames)
        self.pillarMotor = MOTOR("Base_Pillar", numFrames)

    def Tilt(self, t):
        #print("tilting!")
        #desiredAngle = 0.5 #22.5 #10 * math.sin(t / self.fps)
        #print("t:", t)
        x = (t / self.fps) * 2
        #print("x:", x)
        sin = -1 * math.sin(x)
        #print("sine:", sin)
        self.motor.Set_Value(sin / 8, self.platformId)
        self.pillarMotor.Set_Value(0, self.platformId)
