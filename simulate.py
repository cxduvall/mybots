import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np
import math
import random

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")

n = 600
amplitude = math.pi/4
frequency = 10
phaseOffset = 0

pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = np.zeros(n)
frontLegSensorValues = np.zeros(n)

targetAngles = (math.pi/4) * np.sin(np.linspace(phaseOffset, 2*np.pi*frequency + phaseOffset, n))
#np.save("data/targetAngles.npy", targetAngles)

for i in range(n):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_BackLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = targetAngles[i],
        maxForce = 500)

    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_FrontLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = targetAngles[i],
        maxForce = 500)
    
    time.sleep(1/60)
    if i % 60 == 0:
        print("time: " + str(i/60))
np.save("data/frontLegSensorValues.npy", frontLegSensorValues)
np.save("data/backLegSensorValues.npy", backLegSensorValues)

p.disconnect()
