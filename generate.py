import pyrosim.pyrosim as pyrosim

length = 1
width = 1
height = 1
x = 0
y = 0
z = 0.5
scale = 1

def Create_World():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[x-2,y+2,z] , size=[width*scale,length*scale,height*scale])
    pyrosim.End()

def Create_Robot():
    pyrosim.Start_URDF("body.urdf")
    pyrosim.Send_Cube(name="Link0", pos=[x,y,z] , size=[width*scale,length*scale,height*scale])
    pyrosim.Send_Joint( name = "Link0_Link1" , parent= "Link0" , child = "Link1" , type = "revolute", position = [x,y,z+0.5])
    pyrosim.Send_Cube(name="Link1", pos=[0,0,0.5] , size=[width*scale,length*scale,height*scale])
    pyrosim.End()

Create_World()
Create_Robot()


