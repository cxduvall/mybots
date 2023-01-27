import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import math

class WORLD:
    def __init__(self, worldId, fps):
        self.fps = fps
        self.planeId = p.loadURDF("plane.urdf", globalScaling = 0.013)
        #self.planeId2 = p.loadURDF("plane.urdf", globalScaling = 0.02)
        #self.planeId3 = p.loadURDF("plane.urdf", globalScaling = 0.02)
        #self.platformId = p.loadURDF("platform" + worldId + ".urdf")
        self.worldId = p.loadSDF("world" + worldId + ".sdf")
        #pyrosim.Prepare_To_Simulate(self.worldId)
        #p.resetBasePositionAndOrientation(self.planeId, [0,-5,0], [0,0,0,1])
        #p.resetBasePositionAndOrientation(self.planeId2, [0,5.5,0], [0,0,0,1])
        #p.resetBasePositionAndOrientation(self.planeId3, [0,11,0], [0,0,0,1])

        self.set_angular_speed(1,0,0)


    def get_orientation(self):
        return p.getBasePositionAndOrientation(self.planeId)

    def set_angular_speed(self, x, y, z):
        p.resetBaseVelocity(self.planeId, [0,0,0], [x,y,z])

    def Tilt(self, t):
        #print("tilting!")
        #desiredAngle = 0.5 #22.5 #10 * math.sin(t / self.fps)
        #print("t:", t)
        x = (t / self.fps) * 2
        #print("x:", x)
        sin = -1 * math.cos(x) * 2
        #print("sine:", sin)
        angSpeed = sin
        #print("angular speed: ", angSpeed)
        self.set_angular_speed(angSpeed,0,0)
