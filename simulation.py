import pybullet as p
import pyrosim.pyrosim as pyrosim
import pybullet_data
from world import WORLD
from robot import ROBOT
import constants as c
import time
import math

class SIMULATION:
    def __init__(self, directOrGUI, id):
        if directOrGUI == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
        self.directOrGUI = directOrGUI
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        #pyrosim.Prepare_To_Simulate(id)
        self.numFrames = math.floor(c.numSecs * c.fps)
        self.world = WORLD()
        self.robot = ROBOT(self.numFrames, id)
        p.setGravity(0,0, -1 * c.gravityStrength)

    def Run(self):
        for i in range(self.numFrames): # DONE
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think(i)
            self.robot.Act(i)
    
            if self.directOrGUI == "GUI":
                time.sleep(1/c.fps)

            #if i % c.fps == 0:
                #print("time: " + str(i/c.fps))
    
    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def __del__(self):
        p.disconnect()
