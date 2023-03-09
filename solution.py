import numpy as np
import os
import pyrosim.pyrosim as pyrosim
import random
import time
import constants as c
import random
import math
import sys
import csv
import json
import copy

class Cube:
    def __init__(self, parentJIndex=-1, jointPos=[0,0,0],
        globPos=[0,0,0], lwh=[0.5,0.5,0.5]):
        self.parentJIndex = parentJIndex
        self.jointPos = jointPos
        self.globPos = globPos
        self.lwh = lwh

    def print(self):
        # implement if needed - indices?
        pass

class Joint:
    # note that xyz correspond to faceDim 0,1,2 respectively
    def __init__(self, jointPos=[0,0,0], globPos=[0,0,0],
        axis=[1.0,0.0,0.0], parentCubeIndex=0, faceDim=0,
        faceDir=1):
        self.jointPos = jointPos
        self.globPos = globPos
        self.axis = axis
        self.parentCubeIndex = parentCubeIndex
        self.faceDim = faceDim
        self.faceDir = faceDir

    def print(self):
        #implement if needed - indices?
        pass

def GetChildCubeIndex(cubes, k):
        # which index cube has joint k as parent?
        for q in range(len(cubes)):
            cube = cubes[q]
            if cube.parentJIndex == k:
                return q
        return -1

def Write_urdf(myID, cubes, sensors, joints):
    pyrosim.Start_URDF("body" + str(myID) + ".urdf")

    for j in range(len(cubes)):
        name = "Link"+str(j)
        pos = cubes[j].jointPos

        if sensors[j]:
            pyrosim.Send_Cube(name=name, pos=pos, size=cubes[j].lwh,
                matName="Green", matRGBA=[0.0,1.0,0.0,1.0])
        else:
            pyrosim.Send_Cube(name=name, pos=pos, size=cubes[j].lwh,
                matName="Blue", matRGBA=[0.0,0.5,1.0,1.0])

    for k in range(len(joints)):
        parentInd = joints[k].parentCubeIndex
        childInd = GetChildCubeIndex(cubes, k)
        parentName = "Link" + str(parentInd)
        childName = "Link" + str(childInd)
        name = parentName + "_" + childName
        pos = joints[k].jointPos

        pyrosim.Send_Joint(name,parentName,childName,"revolute",pos,jointAxis=joints[k].axis)
    
    pyrosim.End()

def Write_nndf(myID, pidNeurons, sensors, numSensors, numPidNeurons,
    cubes, joints, jointParentLinks, jointChildLinks,
    inputWeights, outputWeights):
    pyrosim.Start_NeuralNetwork("brain" + str(myID) + ".nndf")

    j = 0

    # LAYER 1

    # pidNeurons are sent first...first 3 rows.
    if pidNeurons:
        for dim in ["X", "Y", "Z"]:
            pyrosim.Send_Controlled_Neuron(name = j, controlAttrib = "proportional"+str(dim))
            j += 1
            pyrosim.Send_Controlled_Neuron(name = j, controlAttrib = "integral"+str(dim))
            j += 1
            pyrosim.Send_Controlled_Neuron(name = j, controlAttrib = "derivative"+str(dim))
            j += 1

    for i in range(len(cubes)):
        if sensors[i] == 1:
            # THIS ONLY ADDS SENSOR CUBES (as it should)
            pyrosim.Send_Sensor_Neuron(name = j, linkName = "Link"+str(i))
            j += 1

    for i in range(numSensors):
        pyrosim.Send_Hidden_Neuron(name = j)
        j += 1

    for currentRow in range(numSensors + numPidNeurons):
        for currentColumn in range(numSensors):
            pyrosim.Send_Synapse(sourceNeuronName = currentRow,
                                    targetNeuronName = currentColumn+numSensors+numPidNeurons,
                                    weight = inputWeights[currentRow][currentColumn])

    # LAYER 2

    for i in range(len(joints)):
        pyrosim.Send_Motor_Neuron(name = j,
            jointName="Link"+str(jointParentLinks[i])+"_Link"+str(jointChildLinks[i]))
        j += 1

    prevNeurons = numPidNeurons + numSensors
    for currentRow in range(numSensors):
        for currentColumn in range(len(joints)):
            pyrosim.Send_Synapse(sourceNeuronName = currentRow+prevNeurons,
                                    targetNeuronName = currentColumn+numSensors+prevNeurons, 
                                    weight = outputWeights[currentRow][currentColumn])

    pyrosim.End()

class SOLUTION:
    def __init__(self, id, randseed, bodyEv, pidNeurons):
        self.myID = id
        self.minLinks = 3
        self.maxLinks = 5
        self.body_mutation_rate = 0.3
        self.sensorProbability = 0.8
        self.addCubeProbability = 0.1
        self.removeCubeProbability = 0.9 # must be between 0 and 1 - addCubeProbability
        self.seed_random(randseed)
        self.maxHeight = 3
        self.minL = 0.3
        self.minW = 0.3
        self.minH = 0.3
        self.maxL = 2
        self.maxW = 2
        self.maxH = 2
        self.actualBodyMutations = 0
        self.pidNeurons = pidNeurons
        self.numPidNeurons = 9 if self.pidNeurons else 0
        self.bodyEv = bodyEv

    def Start_Simulation(self, command, direct=True, numSecs=c.numSecs):
        # command can be "create", "mutate", or "view"

        self.Create_World()
        if command == "create":
            self.Create_Body()
            self.Create_Brain()
        elif command == "mutate":
            self.Mutate()
        else: # command == "view"
            self.write_urdf()
            self.write_nndf()

        #print("about to syscall simulate.py")
        if direct:
            directOrGUI = " DIRECT "
        else:
            directOrGUI = " GUI "
        command = "python3 simulate.py " + str(numSecs) + directOrGUI \
            + str(self.myID) + " " + str(self.pidNeurons) + " 2&>sim" # 2&>1 &
        os.system(command)

    def Wait_For_Simulation_To_End(self, direct=True):
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        
        with open(fitnessFileName, "r") as f:
            self.fitness = float(f.readline().strip())
        os.system("rm fitness" + str(self.myID) + ".txt")
        os.system("rm world" + str(self.myID) + ".sdf")
        #print(self.fitness)
        return self.fitness

    def Get_Fitness(self):
        if self.fitness is not None:
            return self.fitness

    def Evaluate(self, command, direct=True, viewLen=c.numSecs):
        self.Start_Simulation(command, direct=direct, numSecs=viewLen)
        self.Wait_For_Simulation_To_End(direct)

    def Set_ID(self, id):
        self.myID = id

    def adjustChildJointPos(self, cubeIndex, cubesCopy, jointsCopy):
        # TODO assume cubesCopy has the correct lwh, jointPos, globPos for the cube, but jointsCopy isn't updated accordingly -- just adjust the jointPos/globPos for the child joints.
        pass

    def recalc_globPos(self, cubeIndex, cubesCopy, jointsCopy):
        # TODO cube lwh already modified in cubesCopy
        # TODO cube jointPos already adjusted in cubesCopy
        # TODO adjust its jointPos
        # TODO after this, if returns [cubesCopy, jointsCopy], commit the changes -- otherwise (returned None) yielded an overlap or height exceeded, and should try again
        pass

    def mutate_brain(self):
        # tweak a single neuron
        num_input_synapses = (self.numSensors + self.numPidNeurons) * self.numSensors
        num_output_synapses = self.numSensors * self.numJoints
        layer_to_tweak = self.random.choices(["input", "output]"], weights=[num_input_synapses,
                                                                            num_output_synapses])

        if layer_to_tweak == "input":
            randomRow = self.random.randint(0, self.numSensors + self.numPidNeurons - 1)
            randomColumn = self.random.randint(0, self.numSensors - 1)
            self.inputWeights[randomRow,randomColumn] = self.random.random() * 2 - 1

        else:
            randomRow = self.random.randint(0, self.numSensors - 1)
            randomColumn = self.random.randint(0, self.numJoints - 1)
            self.outputWeights[randomRow,randomColumn] = self.random.random() * 2 - 1

    def add_cube_mutation(self):
        #print("\nadding a cube...")
        origSensors = self.numSensors
        self.sensors.append(0) # updated below
        # add cube and its joint to respective arrs -- updates self.sensors and self.numSensors redundantly
        while True:
            # TODO handle sensor removal
            sensorNum = self.numSensors
            self.create_joint()
            cubeSuccess = self.create_cube(-1) # chooses if sensor...
            if (not cubeSuccess) or self.get_height() > self.maxHeight:
                self.cubes.pop(-1)
                self.joints.pop(-1)
                self.numSensors = sensorNum
                self.sensors[-1] = 0
            else:
                break
        self.adjustZPositions()
        self.numLinks += 1
        self.numJoints += 1
        self.add_joint_parents_children()
        #print("adding add_joint to mutate_ops")
        self.mutate_ops.append("add_joint")
        # only if added a SENSOR cube!
        if self.numSensors != origSensors:
            #print("adding add_sensor_link to mutate ops")
            self.mutate_ops.append("add_sensor_link")
        # TODO if append a sensor link (or not), do the added link/joint numbers correspond??

        #print("\n---actually added cube!")
        self.actualBodyMutations += 1

    def rm_indices(self, arr, toDel):
        origIndex = 0
        arrIndex = 0
        while origIndex < len(toDel):
            if toDel[origIndex] == 1:
                arr.pop(arrIndex)
                arrIndex -= 1
            arrIndex += 1
            origIndex += 1

    def rm_joint_parents_children(self, cToDel, jToDel):
        self.rm_indices(self.jointParentLinks, jToDel)
        self.rm_indices(self.jointChildLinks, jToDel)
        # TODO adjust entries of jointParentLinks and jointChildLinks for cube re-arrangement
        for i in range(len(self.jointParentLinks)):
            self.jointParentLinks[i] = self.adjustIndex(self.jointParentLinks[i], cToDel)
        for i in range(len(self.jointChildLinks)):
            self.jointChildLinks[i] = self.adjustIndex(self.jointChildLinks[i], cToDel)

    def adjustIndex(self, index, deletedFlags):
        # TODO inefficient -- calculate an arr w sums once instead of per cube/joint if needed
        priorSum = 0
        for i in range(index):
            priorSum += deletedFlags[i]
        return index - priorSum

    def remove_cube_mutation(self):
        
        # identify a main cube index to remove -- removed index cannot be 0 (otherwise, return)
        #print("\nSELF.CUBES BEFORE REMOVAL:", self.cubes)
        #if index == 0:
            #print("can't delete: one cube left and/or core link")
        #    return
        if len(self.cubes) == 1:
            return

        #print("actually removing!")

        index = self.random.randint(1, len(self.cubes) - 1)

        # identify a list of all deleted cube indices
        sensorsToDel = [0] * self.numSensors
        #print("\nsensorsToDel length on creation (shud b numSensors):", len(sensorsToDel))
        cToDel = [0] * len(self.cubes)
        cToDel[index] = 1
        # TODO if first cube to del here is a sensor?
        #print("\n\n BEGINNING DEL-STATE ALGO")
        for n in range(len(self.cubes)):
            #print("\nGeneration n of del-slate algo:", n)
            sensorInd = 0
            for m in range(len(self.cubes)):
                cube = self.cubes[m]
                parentJointIndex = cube.parentJIndex
                #print("SELF.jointParentLinks:", self.jointParentLinks)
                #print("parentJointIndex:", parentJointIndex) # TODO can be -1
                if parentJointIndex == -1:
                    continue
                priorCubeInd = self.jointParentLinks[parentJointIndex] # TODO out of range
                if cToDel[priorCubeInd] == 1:
                    cToDel[m] = 1
                if self.sensors[m]:
                    sensorInd += 1
                    if cToDel[m] == 1:
                        sensorsToDel[sensorInd - 1] = 1

        # ABORT DELETION if plan to delete too many cubes!
        # at least 2 cubes must remain after deletion, otherwise abort
        cubesDeleted = 0
        for n in range(len(cToDel)):
            cubesDeleted += 1
        if len(self.cubes) - cubesDeleted < self.minLinks:
            #print("ABORTING DELETION (< minLinks would remain otherwise)")
            return

        # if a cube is getting deleted, then its parent joint is also getting deleted
        jToDel = [0] * len(self.joints)
        for n in range(len(self.cubes)):
            if cToDel[n] == 1:
                jToDel[self.cubes[n].parentJIndex] = 1

        # update self.cubes (cube indices) + self.joints (joint indices) accordingly (changed indices)
        # update self.sensors for indices
        self.rm_indices(self.cubes, cToDel)
        self.rm_indices(self.joints, jToDel)
        self.rm_indices(self.sensors, cToDel)

        # update self.sensors + self.numSensors (b4 changing indices)
        self.numSensors = 0
        for i in range(len(self.sensors)):
            self.numSensors += self.sensors[i]
        #print("just updated self.numSensors:", self.numSensors)

        # TODO update cube parent joints, and joint parent cubes accordingly
        for cube in self.cubes:
            cube.parentJIndex = self.adjustIndex(cube.parentJIndex, jToDel)
        for joint in self.joints:
            joint.parentCubeIndex = self.adjustIndex(joint.parentCubeIndex, cToDel)

        # update self.numLinks, self.numJoints
        self.numLinks = len(self.cubes)
        self.numJoints = len(self.joints)

        # remove_joint_parents_children
        self.rm_joint_parents_children(cToDel, jToDel)

        # mutate_ops
        dictionary = {"cubes": cToDel, "joints": jToDel, "sensors": sensorsToDel}
        #print("adding this to mutate_ops:", dictionary)
        self.mutate_ops.append(dictionary)

        #print("SELF.CUBES AFTER REMOVAL:", self.cubes)
        #print("MUTATE_OPS AFTER CUBE REMOVAL:", self.mutate_ops, "\n")

        #print("\n---removed cube (actually!)")
        self.actualBodyMutations += 1

    def mutate_body_logic(self, mutationNum):

        #print("\nBODY MUTATION NUM", mutationNum)
        #print("old self.sensors:", self.sensors)

        # check for chances for either of the 2 mutation types, and don't do if too many/too few links

        choice = self.random.random()
        if choice < self.addCubeProbability and self.numLinks < self.maxLinks:
            self.add_cube_mutation()
        elif choice < self.addCubeProbability + self.removeCubeProbability:
            # cases w min links already handled by function
            self.remove_cube_mutation()
        else:
            #print("no body mutation after all.")
            self.mutate_ops.append("none")
        # otherwise, no body logic to change.

        #print("new self.sensors:", self.sensors)

    # cubes rows, joints cols
    def add_joint_to_weights(self):
        # just add a column to outputWeights (for the joint's new motor neuron)
        np.random.seed(self.random.randint(0, 2**30))
        new_col = 2 * np.random.rand(self.outputWeights.shape[0], 1)
        new_col -= 1
        self.outputWeights = np.hstack([self.outputWeights, new_col])

    def add_sensor_to_weights(self):
        # TODO add a row *and then* col to inputWeights, *AND* row to outputWeights
        np.random.seed(self.random.randint(0, 2**30))

        new_row = 2 * np.random.rand(1, self.inputWeights.shape[1])
        new_row -= 1
        self.inputWeights = np.vstack([self.inputWeights, new_row])

        new_col = 2 * np.random.rand(self.inputWeights.shape[0], 1)
        new_col -= 1
        self.inputWeights = np.hstack([self.inputWeights, new_col])

        new_row = 2 * np.random.rand(1, self.outputWeights.shape[1])
        new_row -= 1
        self.outputWeights = np.vstack([self.outputWeights, new_row])

    def rm_sensors_from_weights(self, sensorsToDel):
        # rm: corresponding *adjusted* (by 9) inRow, inCol, outRow, 
        def rm_input_weight_row(k):
            self.inputWeights = np.delete(self.inputWeights, k, 0)

        def rm_input_weight_col(k):
            self.inputWeights = np.delete(self.inputWeights, k, 1)

        def rm_output_weight_row(k):
            self.outputWeights = np.delete(self.outputWeights, k, 0)

        origIndex = 0
        arrIndex = 0
        while origIndex < self.inputWeights.shape[0]:
            if sensorsToDel[origIndex] == 1:
                rm_input_weight_row(arrIndex)
                arrIndex -= 1
            arrIndex += 1
            origIndex += 1

        origIndex = 0
        arrIndex = 0
        while origIndex < self.inputWeights.shape[1]:
            if sensorsToDel[origIndex] == 1:
                rm_input_weight_col(arrIndex)
                arrIndex -= 1
            arrIndex += 1
            origIndex += 1

        origIndex = 0
        arrIndex = 0
        while origIndex < self.outputWeights.shape[0]:
            if sensorsToDel[origIndex] == 1:
                rm_output_weight_row(arrIndex)
                arrIndex -= 1
            arrIndex += 1
            origIndex += 1

    def rm_joints_from_weights(self, jointsToDel):
        # just remove corresponding columns from the output weights
        def rm_output_weight_col(k):
            self.outputWeights = np.delete(self.outputWeights, k, 1)

        origIndex = 0
        arrIndex = 0
        while origIndex < self.outputWeights.shape[1]:
            if jointsToDel[origIndex] == 1:
                rm_output_weight_col(arrIndex)
                arrIndex -= 1
            arrIndex += 1
            origIndex += 1

    def del_from_weights(self, mutation):
        jToDel = mutation["joints"]
        sensorsToDel = mutation["sensors"]

        # TODO mutation "sensors" is 2-len when weights has 3 rows?

        self.rm_sensors_from_weights(sensorsToDel)
        self.rm_joints_from_weights(jToDel)

    def update_brain_logic(self):
        #print("\n\n UPDATING BRAIN LOGIC")
        # use mutate_ops to update weights
        for i in range(len(self.mutate_ops)):
            mutation = self.mutate_ops[i]
            if mutation == "add_joint":
                self.add_joint_to_weights()
            elif mutation == "add_sensor_link":
                self.add_sensor_to_weights()
            elif mutation == "none":
                pass
                #print("\nfound a none-mutation, continuing.")
            else:
                self.del_from_weights(mutation)

        # self.numSensors and self.numJoints have been updated at this point
        # (as well as the size of both weight matrices, in ways corresponding to the
        # potential addition(s) and/or removal(s))

    def Mutate(self):
        self.mutate_ops = []
        mutationNum = 0
        
        rand = self.random.random()
        #print("\nRAND:", rand, "BODY MUTATION RATE:", self.body_mutation_rate)
        if self.bodyEv and rand < self.body_mutation_rate:
            self.mutate_body_logic(mutationNum)
            mutationNum += 1
        
        self.write_urdf()
        self.update_brain_logic()
        self.mutate_brain()
        self.write_nndf()

    def Create_World(self):
        pyrosim.Start_SDF("world" + str(self.myID) + ".sdf")
        pyrosim.End()

    def jointPosOfJointOnCube(self, faceDim, faceDir,
                    cubeLwh, cubeJointPos, isRandom = True):
        l, w, h = cubeLwh
        x, y, z = cubeJointPos
        if faceDim == "x":
            x += (l/2)*faceDir
            if isRandom:
                y += ((self.random.random() * 2) - 1) * (w/2)
                z += ((self.random.random() * 2) - 1) * (h/2)
        elif faceDim == "y":
            y += (w/2)*faceDir
            if isRandom:
                x += ((self.random.random() * 2) - 1) * (l/2)
                z += ((self.random.random() * 2) - 1) * (h/2)
        else:
            if isRandom:
                x += ((self.random.random() * 2) - 1) * (l/2)
                y += ((self.random.random() * 2) - 1) * (w/2)
            z += (h/2)*faceDir

        return [x, y, z]
    
    def normalize(self, vec):
        mag = math.sqrt((vec[0]**2) + (vec[1]**2) + (vec[2]**2))
        return [vec[0]/mag, vec[1]/mag, vec[2]/mag]

    def vecAdd(self, v1, v2):
        return [v1[0]+v2[0], v1[1]+v2[1], v1[2]+v2[2]]

    def findLowestCubeBottom(self):
        minZ = self.cubes[0].globPos[2] - (self.cubes[0].lwh[2]/2)
        for i in range(len(self.cubes)):
            z = self.cubes[i].globPos[2] - (self.cubes[i].lwh[2]/2)
            if z < minZ:
                minZ = z
        return minZ

    def findHighestCubeTop(self):
        maxZ = self.cubes[0].globPos[2] + (self.cubes[0].lwh[2]/2)
        for i in range(len(self.cubes)):
            z = self.cubes[i].globPos[2] + (self.cubes[i].lwh[2]/2)
            if z > maxZ:
                maxZ = z
        return maxZ

    def getChildCubeIndex(self, k):
        return GetChildCubeIndex(self.cubes, k)

    def create_joint(self):
        faceDim = self.random.choice(["x","y","z"])
        faceDir = self.random.choice([-1,1])
        #on a random previous cube index
        #print("SELF.CUBES:", self.cubes) # TODO why are there sometime no cubes?
        prevCubeIndex = self.random.randint(0, len(self.cubes)-1)
        # random position on the face
        jointPosOfNewJoint = self.jointPosOfJointOnCube(faceDim, faceDir,
            self.cubes[prevCubeIndex].lwh, 
            self.cubes[prevCubeIndex].jointPos)

        jointAxis = self.random.choice([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        #print("jointAxis:", jointAxis)

        prevJointInd = self.cubes[prevCubeIndex].parentJIndex

        globPosOfNewJoint = self.vecAdd(jointPosOfNewJoint,
            self.joints[prevJointInd].globPos if prevJointInd != -1 else [0,0,0])
        newJointAxis = " ".join(list(map(str, jointAxis)))

        # create the new joint itself
        self.joints.append(Joint(jointPos=jointPosOfNewJoint,
            globPos = globPosOfNewJoint,
            axis=newJointAxis,
            parentCubeIndex=prevCubeIndex,
            faceDim=faceDim,
            faceDir=faceDir))

    # inspired by https://stackoverflow.com/questions/3269434/whats-the-most-efficient-way-to-test-if-two-ranges-overlap
    # (StartA <= EndB) and (EndA >= StartB)
    def intervals_overlap(self, start1, end1, start2, end2):
        return (start1 <= end2) and (end1 >= start2)

    def cubes_overlap(self, globPos1, lwh1, globPos2, lwh2):
        x1, y1, z1 = globPos1
        x2, y2, z2 = globPos2
        l1, w1, h1 = lwh1
        l2, w2, h2 = lwh2
        x_overlap = self.intervals_overlap(x1-l1/2, x1+l1/2,
            x2-l2/2, x2+l2/2)
        y_overlap = self.intervals_overlap(y1-w1/2, y1+w1/2,
            y2-w2/2, y2+w2/2)
        z_overlap = self.intervals_overlap(z1-h1/2, z1+h1/2,
            z2-h2/2, z2+h2/2)
        return x_overlap and y_overlap and z_overlap

    def overlaps_with_any_existing_cube(self, globPos, lwh):
        for cube in self.cubes:
            if self.cubes_overlap(globPos, lwh, cube.globPos, cube.lwh):
                return True
        return False

    # return True if works, false if didn't bc of overlap
    def create_cube(self, i):

        # now generate info about the cube!!
        if i == 0 or self.random.uniform(0,1) < self.sensorProbability:
            #print("generated a SENSOR cube!")
            self.sensors[i] = 1
            self.numSensors += 1
        #else:
            #print("generated non-sensor cube.")
        l = self.random.uniform(self.minL, self.maxL)
        w = self.random.uniform(self.minW, self.maxW)
        h = self.random.uniform(self.minH, self.maxH)
        
        # add the actual cube
        if i != 0:

            # cubeParentJIndexArr
            parentJIndex = len(self.joints)-1
            # assume last-added joint is to this cube

            # cubeJointPosArr -- make cube joint-pos according to lwh + dim/dir of parent j
            # use parent dim/dir, and new lwh
            cubeJointPos = self.jointPosOfJointOnCube(self.joints[-1].faceDim, self.joints[-1].faceDir,
                [l,w,h], [0,0,0], isRandom=False)
            # treat the joint as a cube centered at itself, lol

            # cubeGlobPosArr -- global position -- in terms of parent joint global position
            cubeGlobPos = self.vecAdd(cubeJointPos, self.joints[parentJIndex].globPos)

            lwh = [l,w,h]

            if self.overlaps_with_any_existing_cube(cubeGlobPos, lwh):
                self.cubes.append("OVERLAP")
                return False
            else:
                self.cubes.append(Cube(lwh=lwh,
                    jointPos=cubeJointPos,
                    globPos=cubeGlobPos,
                    parentJIndex=parentJIndex))
                return True
        else: # first cube, no parent joint
            self.cubes.append(Cube(lwh=[l,w,h],
                jointPos=[0,0,0],
                globPos=[0,0,0],
                parentJIndex=-1))

            return True

    def adjustZPositions(self):
        # adjust global positions of joints/cubes (and joint pos of first cube and its joint(s)) for min z of cube bottoms
        minZ = self.findLowestCubeBottom()
        addZ = 0
        aboveGround = 0.5
        if minZ < aboveGround:
            addZ = aboveGround - minZ
        # actually add to j/c global positions (and joint pos of first cube and its joint(s))
        # involved arrs: JGlobPos, cubeGlobPosArr, cubeJointPosArr[0], JPos[0]
        for k in range(len(self.joints)):
            self.joints[k].globPos[2] += addZ
        for k in range(len(self.cubes)):
            self.cubes[k].globPos[2] += addZ
        self.cubes[0].jointPos[2] += addZ
        # addZ for all joints on the first cube
        for k in range(len(self.joints)):
            if self.joints[k].parentCubeIndex == 0:
                self.joints[k].jointPos[2] += addZ

    def print_body_info(self):
        #print("\nCUBE INFO:")
        for k in range(len(self.cubes)):
            self.cubes[k].print()
        
        #print("\nJOINT INFO:")
        for k in range(len(self.joints)):
            self.joints[k].print()

    def seed_random(self, randseed):
        #random.randrange(sys.maxsize)
        self.randSeed = randseed#parentRandom.randrange(sys.maxsize)#3384984311908638331 #5933355704700017414 #6910913391251108116 #4075562753625725340 #8513075665796686122 #6150022990742915642 # 6518565134699257740 - 2 together/apart if just 2 parts # 7978656969469434807 for floating + ground when 3-10 links
        #print("\nRANDOM SEED:", self.randSeed, "\n")
        self.random = random.Random(self.randSeed)

    def record_joint_parents_children(self):
        self.jointParentLinks = list(map(lambda j: j.parentCubeIndex, self.joints))

        self.jointChildLinks =  list(map(lambda q: self.getChildCubeIndex(q),
            range(self.numJoints)))

    def add_joint_parents_children(self):
        self.jointParentLinks.append(self.joints[-1].parentCubeIndex)
        self.jointChildLinks.append(self.getChildCubeIndex(self.numJoints - 1))

    def write_urdf(self):
        Write_urdf(self.myID, self.cubes, self.sensors, self.joints)

    def get_height(self):
        return self.findHighestCubeTop() - self.findLowestCubeBottom()

    def initialize_body_logic(self):

        self.numLinks = self.random.randint(self.minLinks, self.maxLinks) #2 #self.rng.randint(3,10)
        self.sensors = [0] * self.numLinks
        self.numSensors = 0
        self.numJoints = self.numLinks - 1

        self.cubes = []
        self.joints = []

        # TODO if create_cube returns false, try again...
        for i in range(self.numLinks):
            #self.overlaps_with_any_existing_cube

            while True:
                if i != 0:
                    self.create_joint()
                sensorNum = self.numSensors
                cubeSuccess = self.create_cube(i)
                if (not cubeSuccess) or self.get_height() > self.maxHeight:
                    if i != 0:
                        self.joints.pop(-1) # only run this line when at least 2 cubes bc maxHeight > what 1 cube can be
                    self.cubes.pop(-1)
                    self.numSensors = sensorNum
                    self.sensors[i] = 0
                else:
                    break

        self.adjustZPositions()
        #self.print_body_info()

        self.record_joint_parents_children()
        
    def Create_Body(self):
        self.initialize_body_logic()
        self.write_urdf()

    def initialize_brain_logic(self):

        np.random.seed(self.random.randint(0, 2**30))
        self.inputWeights = 2*np.random.rand(self.numSensors + self.numPidNeurons,
                                            self.numSensors)
        self.inputWeights -= 1

        self.outputWeights = 2*np.random.rand(self.numSensors,self.numJoints)
        self.outputWeights -= 1

    def write_nndf(self):
        Write_nndf(self.myID, self.pidNeurons, self.sensors, self.numSensors, self.numPidNeurons,
            self.cubes, self.joints, self.jointParentLinks, self.jointChildLinks,
            self.inputWeights, self.outputWeights)

    def Create_Brain(self):
        self.initialize_brain_logic()
        self.write_nndf()

class SolutionEncoder(json.JSONEncoder):
    def convertSolution(self, obj):
        theDict = dict(copy.deepcopy(obj.__dict__))
        for key in theDict.keys():
            #print("key:", key)
            #print("val:", theDict[key])
            if isinstance(theDict[key], random.Random):
                theDict[key] = theDict[key].__dict__
            elif isinstance(theDict[key], np.ndarray):
                theDict[key] = theDict[key].tolist()
            elif isinstance(theDict[key], list) and len(theDict[key]) > 0 and isinstance(theDict[key][0], Cube):
                #print("FOUND A CUBE")
                theDict[key] = [cube.__dict__ for cube in theDict[key]]
            elif isinstance(theDict[key], list) and len(theDict[key]) > 0 and isinstance(theDict[key][0], Joint):
                theDict[key] = [joint.__dict__ for joint in theDict[key]]
        theDict["inputWeights"] = copy.deepcopy(obj.inputWeights).tolist()
        theDict["outputWeights"] = copy.deepcopy(obj.outputWeights).tolist()
        theDict["cubes"] = [cube.__dict__ for cube in obj.cubes]
        theDict["joints"] = [joint.__dict__ for joint in obj.joints]
        return theDict

    def default(self, obj):
        #print("TYPE IN ENCODER:", type(obj))
        if isinstance(obj, dict) and isinstance(obj["0"], SOLUTION):
            #print("DICT OF SOLUTIONS IN ENCODER")
            lst = [self.convertSolution(sol) for sol in obj.values()]
            print(lst)
            return [self.convertSolution(sol) for sol in obj.values()]
        elif isinstance(obj, SOLUTION):
            #print("SINGLE SOLUTION IN ENCODER")
            theDict = self.convertSolution(obj)
            return theDict