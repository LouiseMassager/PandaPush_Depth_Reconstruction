import pybullet as p
import os
import time
import pybullet_data
path=os.getcwd()

p.connect(p.GUI_SERVER)

p.loadURDF(path+"/scene1.urdf")


#p.changeConstraint(cid3, maxForce=500.000000)
p.setGravity(0.000000, 0.000000, 0.000000)
while True:
	p.stepSimulation()
	time.sleep(1)
p.disconnect()
