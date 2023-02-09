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

class SOLUTION:
    def __init__(self, id):
        self.myID = id

        self.baseHeight = 0.5
        self.platformHeight = 0.5
        self.platformX = 0
        self.platformY = 0
        self.pillarHeight = 1
        self.platformWidth = 4
        self.platformLength = 4

        self.length = 1
        self.width = 1
        self.height = 1
        self.x = self.platformX
        self.y = self.platformY
        self.z = 0.25 #1.25 + self.baseHeight + self.pillarHeight + self.platformHeight
        self.scale = 1

    def Start_Simulation(self, direct=True):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        #self.Create_Platform()

        print("about to syscall simulate.py")

        if direct:
            command = "python3 simulate.py DIRECT " + str(self.myID) + " 2" # 2&>1 &
        else:
            command = "python3 simulate.py GUI " + str(self.myID) + " 2" # 2&>1 &
        print("Command:", command)
        os.system(command)

    def Wait_For_Simulation_To_End(self, direct=True):
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        
        with open(fitnessFileName, "r") as f:
            self.fitness = float(f.readline().strip())
        os.system("rm fitness" + str(self.myID) + ".txt")
        os.system("rm world" + str(self.myID) + ".sdf")
        print(self.fitness)
        return self.fitness

    def Get_Fitness(self):
        if self.fitness is not None:
            return self.fitness

    def Evaluate(self, direct=True):
        self.Start_Simulation(direct)
        self.Wait_For_Simulation_To_End(direct)

    def Set_ID(self, id):
        self.myID = id

    def Mutate(self):
        randomRow = random.randint(0, self.numSensorNeurons - 1)
        randomColumn = random.randint(0, self.numMotorNeurons - 1)
        self.weights[randomRow,randomColumn] = random.random() * 2 - 1

    def Create_World(self):
        pyrosim.Start_SDF("world" + str(self.myID) + ".sdf")
        #pyrosim.Send_Cube(name="Box", pos=[self.x-2,self.y+2,self.z] , size=[self.width*self.scale,self.length*self.scale,self.height*self.scale])
        pyrosim.End()














    # 1 (a, b); 2 (a, b)
    def overlap(self, a1, b1, a2, b2):
        return (a1 <= b2) and (b1 >= a2)

    # uses global positions
    def get_cube_range(self, pos, lwh ):
        x, y, z = pos
        l, w, h = lwh
        xInt = [x - l/2, x + l/2]
        yInt = [y - w/2, y + w/2]
        zInt = [z - h/2, z + h/2]
        return [xInt, yInt, zInt]
    
    # ranges1: [[x1,x2],[y1,y2],[z1,z2]]
    def cubes_overlap(self, ranges1, ranges2):
        overlapX = self.overlap(ranges1[0][0], ranges1[0][1],
            ranges2[0][0], ranges2[0][1])
        overlapY = self.overlap(ranges1[1][0], ranges1[1][1],
            ranges2[1][0], ranges2[1][1])
        overlapZ = self.overlap(ranges1[2][0], ranges1[2][1],
            ranges2[2][0], ranges2[2][1])
        return overlapX and overlapY and overlapZ

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

    def findLowestCubeBottom(self, cubeGlobalPositions, cubeLWHs):
        minZ = cubeGlobalPositions[0][2] - (cubeLWHs[0][2]/2)
        for i in range(len(cubeGlobalPositions)):
            z = cubeGlobalPositions[i][2] - (cubeLWHs[i][2]/2)
            if z < minZ:
                minZ = z
        return minZ

    def getChildCubeIndex(self, k, cubeParentJIndexArr):
        index = cubeParentJIndexArr.index(k) # which cube has joint k as parent?
        return index

    def Create_Body(self):
        seed = random.randrange(sys.maxsize) #random.randrange(sys.maxsize) # 6518565134699257740 - 2 together/apart if just 2 parts # 7978656969469434807 for floating + ground when 3-10 links
        print("RANDOM SEED:", seed)
        self.random = random.Random(seed)

        pyrosim.Start_URDF("body" + str(self.myID) + ".urdf")

        self.numLinks = self.random.randint(3,10) #2 #self.rng.randint(3,10)
        self.sensors = [0] * self.numLinks
        self.numSensors = 0
        self.numJoints = self.numLinks - 1

        # cube info arrays
        cubeParentJIndexArr = [] # -1 for [0]
        cubeJointPosArr = []
        cubeGlobPosArr = []
        lwhArr = []

        # joint info arrays
        JPos = [] # relative to PARENT JOINT
        JGlobPos = []
        JAxes = []
        JParentCubeIndices = []
        JFaceDims = []
        JFaceDirs = []

        for i in range(self.numLinks):

            # generate a joint on a previous cube (unless 1st cube)
            if i != 0:
                faceDim = self.random.choice(["x","y","z"])
                faceDir = self.random.choice([-1,1])
                #on a random previous cube index
                prevCubeIndex = self.random.randint(0, len(lwhArr)-1)
                # random position on the face
                jointPosOfNewJoint = self.jointPosOfJointOnCube(faceDim, faceDir,
                    lwhArr[prevCubeIndex], cubeJointPosArr[prevCubeIndex])
                
                #'''
                # random joint axis (in *any* direction!)
                jointAxis = []
                for j in range(3):
                    jointAxis.append(self.random.random())
                # normalize the joint axis
                jointAxis = self.normalize(jointAxis)
                #'''
                jointAxis = random.choice([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
                print("jointAxis:", jointAxis)

                prevJointInd = cubeParentJIndexArr[prevCubeIndex]

                # update JPos, JAxes, JParentCubeIndices
                JPos.append(jointPosOfNewJoint)
                JGlobPos.append(self.vecAdd(jointPosOfNewJoint,
                    JGlobPos[prevJointInd] if prevJointInd != -1 else [0,0,0]))
                JAxes.append(" ".join(list(map(str, jointAxis))))
                JParentCubeIndices.append(prevCubeIndex)

                JFaceDims.append(faceDim)
                JFaceDirs.append(faceDir)

            # now generate the cube itself!!
            if self.random.uniform(0,1) > 0.5:
                print("len sensors:", len(self.sensors))
                print("i:", i)
                print("self.numLinks:",self.numLinks)
                self.sensors[i] = 1
                self.numSensors += 1
            l = self.random.uniform(0.5, 4)
            w = self.random.uniform(0.5, 4)
            h = self.random.uniform(0.5, 4)
            lwhArr.append([l,w,h])
            
            # append to cube arrays
            if i != 0:
                # cubeParentJIndexArr
                parentJIndex = len(JPos)-1
                cubeParentJIndexArr.append(parentJIndex) # assume last-added joint is to this cube
                # cubeJointPosArr -- make cube joint-pos according to lwh + dim/dir of parent j
                # use parent dim/dir, and new lwh
                cubeJointPos = self.jointPosOfJointOnCube(JFaceDims[-1], JFaceDirs[-1],
                    [l,w,h], [0,0,0], isRandom=False)
                cubeJointPosArr.append(cubeJointPos) # treat the joint as a cube centered at itself, lol
                # cubeGlobPosArr -- global position -- in terms of parent joint global position
                cubeGlobPosArr.append(self.vecAdd(cubeJointPos, JGlobPos[parentJIndex]))
            else: # first cube, no parent joint
                cubeJointPosArr.append([0,0,0])
                cubeGlobPosArr.append([0,0,0])
                cubeParentJIndexArr.append(-1)

        
        # TODO: adjust global positions of joints/cubes (and joint pos of first cube and its joint(s)) for min z of cube bottoms
        minZ = self.findLowestCubeBottom(cubeGlobPosArr, lwhArr)
        addZ = 0
        aboveGround = 0.5
        if minZ < aboveGround:
            addZ = aboveGround - minZ
        # actually add to j/c global positions (and joint pos of first cube and its joint(s))
        # involved arrs: JGlobPos, cubeGlobPosArr, cubeJointPosArr[0], JPos[0]
        for k in range(self.numJoints):
            JGlobPos[k][2] += addZ
        for k in range(self.numLinks):
            cubeGlobPosArr[k][2] += addZ
        cubeJointPosArr[0][2] += addZ
        # addZ for all joints on the first cube
        for k in range(self.numJoints):
            if JParentCubeIndices[k] == 0:
                JPos[k][2] += addZ
        

        # cube info dict
        cubeInfo = [None] * self.numLinks
        for c in range(self.numLinks):
            cubeInfo[c] = {"parentJ": cubeParentJIndexArr[c],
                "globPos": cubeGlobPosArr[c], "jointPos": cubeJointPosArr[c],
                "lwh": lwhArr[c]}

        jointInfo = [None] * self.numJoints
        for t in range(self.numJoints):
            jointInfo[t] = {"pos": JPos[t], "globPos": JGlobPos[t],
                "axis": JAxes[t], "parentCubeIndex": JParentCubeIndices[t],
                "faceDims": JFaceDims[t], "faceDirs": JFaceDirs[t]}


        print("\nCUBE INFO:", cubeInfo)
        print("\nJOINT INFO:", jointInfo)

        cubeFieldNames = cubeInfo[0].keys()
        with open('cubes_' + str(seed) + '.csv', 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=cubeFieldNames)
            writer.writeheader()
            writer.writerows(cubeInfo)

        jointFieldNames = jointInfo[0].keys()
        with open('joints_' + str(seed) + '.csv', 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=jointFieldNames)
            writer.writeheader()
            writer.writerows(jointInfo)

        # guarantee at least 1 sensor
        if self.numSensors < 1:
            self.sensors[self.random.randint(0, len(self.sensors) - 1)] = 1
            self.numSensors += 1
        print("\nself.numSensors:", self.numSensors)
        
        for j in range(self.numLinks): # self.numLinks
            name = "Link"+str(j)
            pos = cubeJointPosArr[j]

            '''
            if j == 0:
                c = [1.0, 0.0, 0.0]
            elif j == 1:
                c = [0.0, 1.0, 0.0]
            elif j == 2:
                c = [0.0, 0.0, 1.0]
            elif j == 3:
                c = [0.0, 0.0, 0.0]
            elif j == 4:
                c = [1.0, 1.0, 1.0]

            if j == 0 or j == 3:
                pyrosim.Send_Cube(name=name, pos=pos, size=lwhArr[j],
                        matName=str(j), matRGBA=[c[0],c[1],c[2],1.0])
            '''

            '''
            if j == 0:
                pyrosim.Send_Cube(name=name, pos=pos, size=lwhArr[j],
                    matName="Main", matRGBA=[1.0,0.0,0.0,1.0])
            elif JParentCubeIndices[cubeParentJIndexArr[j]] == 0:
                pyrosim.Send_Cube(name=name, pos=pos, size=lwhArr[j],
                    matName="Secondary", matRGBA=[1.0,0.5,0.5,1.0])
            else:
                pyrosim.Send_Cube(name=name, pos=pos, size=lwhArr[j],
                    matName="Blue", matRGBA=[0.0,0.5,1.0,1.0])
            '''
            #'''
            if self.sensors[j]:
                pyrosim.Send_Cube(name=name, pos=pos, size=lwhArr[j],
                    matName="Green", matRGBA=[0.0,1.0,0.0,1.0])
            else:
                pyrosim.Send_Cube(name=name, pos=pos, size=lwhArr[j],
                    matName="Blue", matRGBA=[0.0,0.5,1.0,1.0])
            #'''

        for k in range(self.numJoints): # self.numJoints #3 # k is a joint-index
            parentInd = JParentCubeIndices[k]
            childInd = self.getChildCubeIndex(k, cubeParentJIndexArr)
            parentName = "Link" + str(parentInd)
            childName = "Link" + str(childInd)
            name = parentName + "_" + childName
            pos = JPos[k]

            #if k == 2:
            pyrosim.Send_Joint(name,parentName,childName,"revolute",pos,jointAxis=JAxes[k])

        # TODO remove

        self.jointParentLinks = JParentCubeIndices
        self.jointChildLinks = list(map(lambda q: self.getChildCubeIndex(q,cubeParentJIndexArr),
            range(self.numJoints)))

        print("\nCube Global positions:", cubeGlobPosArr)
        print("\nCube joint positions:", cubeJointPosArr)
        print("LWHs:", lwhArr)
        print("Joint positions:", JGlobPos)

        pyrosim.End()

    # TODO
    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        j = 0
        numSensorNeurons = 0
        for i in range(self.numLinks):
            #if self.sensors[i] and (i == 0 or i == 3): # TODO rm
            pyrosim.Send_Sensor_Neuron(name = j, linkName = "Link"+str(i))
            j += 1
            numSensorNeurons += 1

        for i in range(self.numJoints):
            #if i == 2: # TODO rm
            pyrosim.Send_Motor_Neuron( name = j,
                jointName="Link"+str(self.jointParentLinks[i])+"_Link"+str(self.jointChildLinks[i]))
            j += 1

        numMotorNeurons = self.numLinks - 1

        self.numSensorNeurons = numSensorNeurons
        self.numMotorNeurons = numMotorNeurons
        self.weights = 2*np.random.rand(self.numSensorNeurons,self.numMotorNeurons)
        self.weights -= 1

        for currentRow in range(numSensorNeurons): # sensor
            for currentColumn in range(numMotorNeurons): # motor
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+numSensorNeurons, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()
