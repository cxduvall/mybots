import pybullet as p
import pyrosim.pyrosim as pyrosim
import pybullet_data
from world import WORLD
from robot import ROBOT
import constants as c
import time
import math

class SIMULATION:
    def __init__(self, directOrGUI, id, numSecs):
        if directOrGUI == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
            p.resetDebugVisualizerCamera( cameraDistance=10, cameraYaw=45, cameraPitch=-45, cameraTargetPosition=[0,0,0])
        self.directOrGUI = directOrGUI
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        #print("preparing to simulate! id:", id)
        #pyrosim.Prepare_To_Simulate(id) # doesn't make sense...load, then sim w *that* id
        #print("success!")
        self.numFrames = math.floor(numSecs * c.fps)
        self.world = WORLD(id, c.fps)
        self.robot = ROBOT(self.numFrames, id)
        #print("ABOUT TO MAKE PLATFORM")
        #self.platform = PLATFORM(self.numFrames, id, c.fps)
        #print("MADE PLATFORM. !!!")
        p.setGravity(0,0, -1 * c.gravityStrength)

    def Run(self):
        #print("actually running simulation!")
        for i in range(self.numFrames): # DONE
            #print("step:", i)

            #print("PLANE POS:", self.world.get_oritentation())
            #self.world.set_angular_speed(0.1,0,0)

            p.stepSimulation()

            #self.platform.Prepare()
            #self.world.Tilt(i)

            self.robot.Prepare()
            self.robot.Sense(i)
            #print("thought:", i)
            self.robot.Think(i)
            #print("sensed:", i)
            self.robot.Act(i)
            #print("acted:", i)
    
            if self.directOrGUI == "GUI":
                time.sleep(1/c.fps)

            #if i % c.fps == 0:
                #print("time: " + str(i/c.fps))
    
    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def __del__(self):
        p.disconnect()
