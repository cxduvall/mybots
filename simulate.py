import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np
import math
import random
import constants as c
from simulation import SIMULATION

'''
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-1 * c.gravityStrength)

planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)

n = c.numSecs * c.fps

backLegSensorValues = np.zeros(n)
frontLegSensorValues = np.zeros(n)

backTargetAngles = c.backAmplitude * np.sin(np.linspace(c.backPhaseOffset, 2*np.pi*c.backFrequency + c.backPhaseOffset, n))
#np.save("data/backTargetAngles.npy", backTargetAngles)

frontTargetAngles = c.frontAmplitude * np.sin(np.linspace(c.frontPhaseOffset, 2*np.pi*c.frontFrequency + c.frontPhaseOffset, n))
#np.save("data/frontTargetAngles.npy", frontTargetAngles)

# ^ DONE

for i in range(n): # DONE
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_BackLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = backTargetAngles[i],
        maxForce = c.maxForce)

    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_FrontLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = frontTargetAngles[i],
        maxForce = c.maxForce)
    
    time.sleep(1/c.fps)
    if i % c.fps == 0:
        print("time: " + str(i/c.fps))




np.save("data/frontLegSensorValues.npy", frontLegSensorValues)
np.save("data/backLegSensorValues.npy", backLegSensorValues)

p.disconnect() # DONE
'''

simulation = SIMULATION()
simulation.Run()
