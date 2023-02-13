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

class SOLUTION:
    def __init__(self, id, parentRandom):
        self.myID = id
        self.minLinks = 3
        self.maxLinks = 10
        self.body_mutation_rate = 0.3
        self.sensorProbability = 0.5
        self.addCubeProbability = 0.1
        self.removeCubeProbability = 0.4 # must be between 0 and 1 - addCubeProbability
        self.parentRandom = parentRandom
        self.maxHeight = 7

    def Start_Simulation(self, direct=True, firstGen=True):

        self.Create_World()
        if firstGen:
            self.Create_Body(self.parentRandom)
            self.Create_Brain()
        else:
            self.Mutate()

        #print("about to syscall simulate.py")
        if direct:
            command = "python3 simulate.py DIRECT " + str(self.myID) + " 2" # 2&>1 &
        else:
            command = "python3 simulate.py GUI " + str(self.myID) + " 2" # 2&>1 &
        #print("Command:", command)
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

    def Evaluate(self, direct=True):
        self.Start_Simulation(direct)
        self.Wait_For_Simulation_To_End(direct)

    def Set_ID(self, id):
        self.myID = id

    def mutate_brain(self):
        # tweak a neuron
        randomRow = self.random.randint(0, self.numSensors - 1)
        randomColumn = self.random.randint(0, self.numJoints - 1)
        #print("\nself.cubes:", self.cubes)
        #print("\nself.joints:", self.joints)
        #print("\nself.sensors:", self.sensors)
        #print("\nself.numSensors:", self.numSensors)
        #print("\nself.numJoints:", self.numJoints)
        #print("\nself.weights.shape:", self.weights.shape)
        self.weights[randomRow,randomColumn] = self.random.random() * 2 - 1 # TODO out of bounds

    def add_cube_mutation(self):
        #print("\nadding a cube...")
        origSensors = self.numSensors
        self.sensors.append(0) # updated below
        # add cube and its joint to respective arrs -- updates self.sensors and self.numSensors redundantly
        while True:
            self.create_joint()
            self.create_cube(-1) # chooses if sensor...
            if self.get_height() > self.maxHeight:
                self.cubes.pop(-1)
                self.joints.pop(-1)
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
        index = self.random.randint(0, len(self.cubes) - 1)
        #print("\nSELF.CUBES BEFORE REMOVAL:", self.cubes)
        if index == 0:
            #print("can't delete: one cube left and/or core link")
            return

        # identify a list of all deleted cube indices
        sensorsToDel = [0] * self.numSensors
        #print("\nsensorsToDel length on creation (shud b numSensors):", len(sensorsToDel))
        #print("self.weights.shape on sensorsToDel creation:", self.weights.shape)
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
        #print("old weights:", self.weights)
        np.random.seed(self.random.randint(0, 2**30))
        new_col = 2 * np.random.rand(self.weights.shape[0], 1)
        new_col -= 1
        self.weights = np.hstack([self.weights, new_col])
        #print("new weights:", self.weights)

    def add_link_to_weights(self):
        #print("old weights:", self.weights)
        np.random.seed(self.random.randint(0, 2**30))
        new_row = 2 * np.random.rand(1, self.weights.shape[1])
        new_row -= 1
        self.weights = np.vstack([self.weights, new_row])
        #print("new weights:", self.weights)

    def rm_weight_rows(self, rowsToDel):
        def rm_weight_row(k):
            self.weights = np.delete(self.weights, k, 0)

        origIndex = 0
        arrIndex = 0
        #print("\nrm_weight_rows - self.weights.shape:", self.weights.shape)
        #print("rm_weight_rows - rowsToDel:", rowsToDel)
        while origIndex < self.weights.shape[0]:
            if rowsToDel[origIndex] == 1: # TODO out of range
                rm_weight_row(arrIndex)
                arrIndex -= 1
            arrIndex += 1
            origIndex += 1

    def rm_weight_cols(self, colsToDel):
        def rm_weight_col(k):
            self.weights = np.delete(self.weights, k, 1)

        origIndex = 0
        arrIndex = 0
        #print("\nrm_weight_cols - self.weights.shape:", self.weights.shape)
        #print("rm_weight_cols - colsToDel:", colsToDel)
        while origIndex < self.weights.shape[1]:
            if colsToDel[origIndex] == 1:
                rm_weight_col(arrIndex)
                arrIndex -= 1
            arrIndex += 1
            origIndex += 1

    def del_from_weights(self, mutation):
        #print("old weights:", self.weights)
        jToDel = mutation["joints"]
        sensorsToDel = mutation["sensors"]

        # TODO mutation "sensors" is 2-len when weights has 3 rows?

        self.rm_weight_rows(sensorsToDel)
        self.rm_weight_cols(jToDel)
        #print("new weights:", self.weights)

    def update_brain_logic(self):
        #print("\n\n UPDATING BRAIN LOGIC")
        # use mutate_ops to update self.weights
        for i in range(len(self.mutate_ops)):
            #print("\nmutation", i)
            mutation = self.mutate_ops[i]
            #print("mutate op:", mutation)
            if mutation == "add_joint":
                self.add_joint_to_weights()
            elif mutation == "add_sensor_link":
                self.add_link_to_weights()
            elif mutation == "none":
                pass
                #print("\nfound a none-mutation, continuing.")
            else:
                self.del_from_weights(mutation)

        # self.numSensors and self.numJoints have been updated at this point (as well as the size of self.weights, in ways corresponding to the potential addition(s) and/or removal(s))

    def Mutate(self):
        self.mutate_ops = []
        mutationNum = 0
        while self.random.random() < self.body_mutation_rate: # TODO make if
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
        # which index cube has joint k as parent?
        for q in range(len(self.cubes)):
            cube = self.cubes[q]
            if cube.parentJIndex == k:
                return q
        return -1

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

    def create_cube(self, i):

        # now generate info about the cube!!
        if i == 0 or self.random.uniform(0,1) < self.sensorProbability:
            #print("generated a SENSOR cube!")
            self.sensors[i] = 1
            self.numSensors += 1
        #else:
            #print("generated non-sensor cube.")
        l = self.random.uniform(0.5, 4)
        w = self.random.uniform(0.5, 4)
        h = self.random.uniform(0.5, 4)
        
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

            self.cubes.append(Cube(lwh=[l,w,h],
                jointPos=cubeJointPos,
                globPos=cubeGlobPos,
                parentJIndex=parentJIndex))
        else: # first cube, no parent joint
            self.cubes.append(Cube(lwh=[l,w,h],
                jointPos=[0,0,0],
                globPos=[0,0,0],
                parentJIndex=-1))

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

    def seed_random(self, parentRandom):
        #random.randrange(sys.maxsize)
        self.randSeed = parentRandom.randrange(sys.maxsize)#3384984311908638331 #5933355704700017414 #6910913391251108116 #4075562753625725340 #8513075665796686122 #6150022990742915642 # 6518565134699257740 - 2 together/apart if just 2 parts # 7978656969469434807 for floating + ground when 3-10 links
        print("\nRANDOM SEED:", self.randSeed, "\n")
        self.random = random.Random(self.randSeed)

    def record_joint_parents_children(self):
        self.jointParentLinks = list(map(lambda j: j.parentCubeIndex, self.joints))

        self.jointChildLinks =  list(map(lambda q: self.getChildCubeIndex(q),
            range(self.numJoints)))

    def add_joint_parents_children(self):
        self.jointParentLinks.append(self.joints[-1].parentCubeIndex)
        self.jointChildLinks.append(self.getChildCubeIndex(self.numJoints - 1))

    def write_urdf(self):
        pyrosim.Start_URDF("body" + str(self.myID) + ".urdf")

        for j in range(len(self.cubes)):
            name = "Link"+str(j)
            pos = self.cubes[j].jointPos

            if self.sensors[j]:
                pyrosim.Send_Cube(name=name, pos=pos, size=self.cubes[j].lwh,
                    matName="Green", matRGBA=[0.0,1.0,0.0,1.0])
            else:
                pyrosim.Send_Cube(name=name, pos=pos, size=self.cubes[j].lwh,
                    matName="Blue", matRGBA=[0.0,0.5,1.0,1.0])

        for k in range(len(self.joints)):
            parentInd = self.joints[k].parentCubeIndex
            childInd = self.getChildCubeIndex(k)
            parentName = "Link" + str(parentInd)
            childName = "Link" + str(childInd)
            name = parentName + "_" + childName # TODO bad indices
            pos = self.joints[k].jointPos

            pyrosim.Send_Joint(name,parentName,childName,"revolute",pos,jointAxis=self.joints[k].axis)
        
        pyrosim.End()

    def get_height(self):
        return self.findHighestCubeTop() - self.findLowestCubeBottom()

    def initialize_body_logic(self, parentRandom):
        self.seed_random(parentRandom)

        self.numLinks = self.random.randint(self.minLinks, self.maxLinks) #2 #self.rng.randint(3,10)
        self.sensors = [0] * self.numLinks
        self.numSensors = 0
        self.numJoints = self.numLinks - 1

        self.cubes = []
        self.joints = []

        for i in range(self.numLinks):
            while True:
                if i != 0:
                    self.create_joint()
                self.create_cube(i)
                if self.get_height() > self.maxHeight:
                    self.joints.pop(-1) # only run this line when at least 2 cubes bc maxHeight > what 1 cube can be
                    self.cubes.pop(-1)
                else:
                    break

        self.adjustZPositions()
        #self.print_body_info()

        self.record_joint_parents_children()
        
    def Create_Body(self, parentRandom):
        self.initialize_body_logic(parentRandom)
        self.write_urdf()

    def initialize_brain_logic(self):
        np.random.seed(self.random.randint(0, 2**30))
        self.weights = 2*np.random.rand(self.numSensors,self.numJoints)
        self.weights -= 1

    def write_nndf(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        j = 0
        for i in range(len(self.cubes)):
            if self.sensors[i] == 1:
                # THIS ONLY ADDS SENSOR CUBES
                pyrosim.Send_Sensor_Neuron(name = j, linkName = "Link"+str(i))
                j += 1

        for i in range(self.numJoints):
            pyrosim.Send_Motor_Neuron(name = j,
                jointName="Link"+str(self.jointParentLinks[i])+"_Link"+str(self.jointChildLinks[i]))
            j += 1

        for currentRow in range(self.numSensors):
            for currentColumn in range(self.numJoints):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+self.numSensors, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()

    def Create_Brain(self):
        self.initialize_brain_logic()
        self.write_nndf()
