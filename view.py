from solution import SOLUTION, Cube, Joint, Write_urdf, Write_nndf
import json
import sys
import os
import time
import numpy as np
import pyrosim.pyrosim as pyrosim

# runSeed viewLen
popToShow = int(sys.argv[1])
viewLen = int(sys.argv[2])
pid = bool(sys.argv[3])
filename = sys.argv[4]

here = path = os.getcwd()
filesHere = os.listdir(here)

file = open(filename, 'r')
parentStr = file.readlines()[0].rstrip("\n")
#print(parentStr)
parents = json.loads(parentStr)
#print("\nparents:", parents)

values = list(parents.values())
for i in range(popToShow):
    id = 0
    parent = values[i]

    # package more complicated attributes

    #print("\n", parent)

    cubes = parent["cubes"]
    for i in range(len(cubes)):
        cubeDict = cubes[i]
        cubes[i] = Cube(**cubeDict)

    joints = parent["joints"]
    for i in range(len(joints)):
        jointDict = joints[i]
        joints[i] = Joint(**jointDict)

    inputWeights = np.array(parent["inputWeights"])
    outputWeights = np.array(parent["outputWeights"])

    # Now write the files

    Write_urdf(id, cubes, parent["sensors"], joints)

    Write_nndf(id, parent["pidNeurons"], parent["sensors"],
        parent["numSensors"], parent["numPidNeurons"],
        cubes, joints, parent["jointParentLinks"],
        parent["jointChildLinks"],
        inputWeights, outputWeights)

    pyrosim.Start_SDF("world" + str(id) + ".sdf")
    pyrosim.End()

    command = "python3 simulate.py " + str(viewLen) + " GUI " \
            + str(id) + " " + str(pid) + " 2" # 2&>1 &
    os.system(command)
