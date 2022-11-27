import open3d as o3d
import numpy as np
import os
import sys
import random

initext='<?xml version="1.0"?>\n<robot name="pandapush">\n'
objtext='\t<link name="{name}">\n\t\t<visual>\n\t\t<origin rpy="{rpy}" xyz="{xyz}"/>\n\t\t<geometry>\n\t\t<mesh scale= ".0010 .0010 .0010" filename="{path}/{name}"/>\n\t\t</geometry>\n\t\t<color rgba="{color}"/>\n\t\t</visual>\n\t</link>\n'
endtext='</robot>'

path=os.getcwd()
files=[['bunny.stl','0 0 0','0 0 0']] #list of object: [stlname,orientation,position,opt:color]
name="scene1.urdf"
scene=open(name,'+w')

def addobject(obj):
	if len(obj)>3:
		color=obj[3]
	else:
		color=''
		for a in range(3):
			color+=str(random.randint(0,101)/100)+' '
		color=color+"1"
	scene.write(objtext.format(name=obj[0],rpy=obj[1],xyz=obj[2],path=path,color=color))
	
	return

########################################################################################################

scene.write(initext)

for obj in files:
	print("object:",end='')
	print(obj)
	addobject(obj)

scene.write(endtext)

scene.close()
