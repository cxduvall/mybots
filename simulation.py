import pybullet as p
import pyrosim.pyrosim as pyrosim
import pybullet_data
from world import WORLD
from robot import ROBOT
import constants as c
import time

class SIMULATION:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.numFrames = c.numSecs * c.fps
        self.world = WORLD()
        self.robot = ROBOT(self.numFrames)
        p.setGravity(0,0,-1 * c.gravityStrength)

    def Run(self):
        for i in range(self.numFrames): # DONE
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Act(i)
    
            time.sleep(1/c.fps)
            if i % c.fps == 0:
                print("time: " + str(i/c.fps))

    def __del__(self):
        p.disconnect()
