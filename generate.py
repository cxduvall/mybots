import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("boxes.sdf")

length = 1
width = 1
height = 1
x = 0
y = 0
z = 0.5


for j in range(5):
    for k in range(5):
        scale = 1
        z = 0.5
        for i in range(5):
            pyrosim.Send_Cube(name="Box" + str(i), pos=[x+j,y+k,z] , size=[width*scale,length*scale,height*scale])
            z += scale/2
            scale *= 0.9
            z += scale/2

pyrosim.End()
