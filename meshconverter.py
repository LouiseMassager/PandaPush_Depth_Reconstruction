import open3d as o3d
import numpy as np
import os
import sys

def display_ply(mesh_name):
	pcd=o3d.io.read_point_cloud(mesh_name)
	print(pcd)
	print(np.asarray(pcd.points))
	o3d.visualization.draw_geometries([pcd])
	return

def list_to_string(liste):
	string=""
	for elem in liste:
		string+=str(elem)+' '
		
	return string[0:-1]

def datalist_to_ply(datalist,mesh_name):
	supp_text="property float32 confidence\nproperty float32 intensity\n"
	supp_value=" 0.900159 0.5"
	
	datafile = open(mesh_name+".ply",'w+') 
	print(mesh_name+".ply")
	pointlist=[]
	facelist=[]
	a=[]
	for face in datalist:
		for point in face:
			if point in pointlist:
				index= pointlist.index(point)
				a.append(index)
			else:
				pointlist.append(point)
				a.append(len(pointlist)-1)
		facelist.append(a)
		a=[]
	print(facelist)
	face_n=len(facelist)
	point_n=len(pointlist)
	datafile.write("ply\nformat ascii 1.0\nelement vertex "+str(point_n)+"\nproperty float32 x\nproperty float32 y\nproperty float32 z\n"+supp_text+"element face "+str(face_n)+"\nproperty list uint8 int32 vertex_indices\nend_header\n")
	for point in pointlist:
		datafile.write(list_to_string(point)+supp_value+"\n")
	for face in facelist:
		datafile.write("3 "+list_to_string(face)+"\n")
	datafile.close()
	return mesh_name+".ply"

def stl_to_ply(mesh_name):
	a=[]
	b=[]
	print("stl to ply")
	f=open(mesh_name,"r+")
	for line in f:
		if 'vertex' in line:
			k=line.index('vertex')
			line=line[k:]
			liste=line.split(' ')
			x=liste[1]
			y=liste[2]
			z=liste[3][0:-1]
			b.append([x,y,z])
			print(b)
			if len(b)==3:
				a.append(b)
				b=[]
	return datalist_to_ply(a,mesh_name[0:-4])

def ply_to_stl(mesh_name):
	print("ply to stl")
	return mesh_name[0:-4]+".stl"


try:
	input1 = str(sys.argv[1]).lower()
except:
	input1="mesh1.stl"
	print("no input")
finally:
	mesh_name = input1


if mesh_name[-4:]==".ply":
	display_ply(mesh_name)
elif mesh_name[-4:]==".stl":
	display_ply(stl_to_ply(mesh_name))
	#stl_to_ply(mesh_name)
