import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("boxes.sdf")

length = 1
width = 1
height = 1
x = 0
y = 0
z = 0.5
pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[width,length,height])
pyrosim.Send_Cube(name="Box2", pos=[x+1,y,z+1] , size=[width,length,height])

pyrosim.End()
