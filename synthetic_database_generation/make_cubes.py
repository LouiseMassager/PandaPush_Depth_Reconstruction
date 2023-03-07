import open3d as o3d
import numpy as np
import os
import sys
import random #random.randint(start, stop+1)

import pymeshlab


#COMMON PARAMETERS
dx=0.01
pert_depth=dx
pert_color=100

#######################################################################################
print("Generate Shapes...")



def ply_to_stl(ply_name):
	ms = pymeshlab.MeshSet()
	ms.load_new_mesh(ply_name+".ply")
	ms.compute_normals_for_point_sets()
	ms.surface_reconstruction_screened_poisson()
	ms.meshing_invert_face_orientation()
	ms.compute_color_transfer_vertex_to_face()
	ms.save_current_mesh(ply_name+".stl")	
	stl_name=ply_name+".stl"
	return stl_name
	
def ply_to_obj(ply_name):
	ms = pymeshlab.MeshSet()
	ms.load_new_mesh(ply_name+".ply")
	ms.compute_normal_for_point_clouds()
	#ms.compute_normals_for_point_sets()
	#ms.surface_reconstruction_screened_poisson()
	#ms.meshing_invert_face_orientation()
	ms.compute_color_transfer_vertex_to_face()
	
	ms.compute_texcoord_by_function_per_vertex()
	ms.compute_texcoord_transfer_vertex_to_wedge()
	ms.compute_texcoord_parametrization_triangle_trivial_per_wedge(textdim = 2048)
	
	ms.compute_color_by_function_per_face()#define material in .mtl file (ref: https://people.sc.fsu.edu/~jburkardt/data/mtl/mtl.html)
	ms.compute_texmap_from_color(textname=f""+ply_name[-5:]+".png")#define color map in .png file
	ms.save_current_mesh(ply_name+".obj")
	obj_name=ply_name+".obj"
	return obj_name


def display_ply(mesh_name):
	pcd=o3d.io.read_point_cloud(mesh_name+".ply")
	print(pcd)
	#print(np.asarray(pcd.points))
	o3d.visualization.draw_geometries([pcd])
	return

def list_to_string(liste):
	string=""
	for elem in liste:
		string+=str(elem)+' '
		
	return string[0:-1]

def datalist_to_ply(datalist,mesh_name,color,norms=None):
	print("\t\tWrite on the ply file")
	comment_text="comment author: Louise Massager\ncomment object: "+mesh_name+"\n"
	color_text="property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\n"
	supp_text=""#"property float32 confidence\nproperty float32 intensity\n"
	supp_value=""#" 0.900159 0.5"
	if norms !=None:
		norm_text="property float nx\nproperty float ny\nproperty float nx\n"
	else:
		norm_text=""
		
	datafile = open(mesh_name+".ply",'w+') 
	pointlist=[]
	facelist=[]
	normlist=[]
	a=[]
	i=0
	for face in datalist:
		for point in face:
			if point in pointlist:
				index= pointlist.index(point)
				a.append(index)
			else:
				pointlist.append(point)
				if norms!=None:
					normlist.append(norms[i])
				a.append(len(pointlist)-1)
		facelist.append(a)
		a=[]
		i+=1
	face_n=len(facelist)
	point_n=len(pointlist)
	datafile.write("ply\nformat ascii 1.0\n"+comment_text+"element vertex "+str(point_n)+"\nproperty float x\nproperty float y\nproperty float z\n"+norm_text+color_text+supp_text+"element face "+str(face_n)+"\nproperty list uint8 int32 vertex_indices\nend_header\n")
	
	for i in range(len(pointlist)):
		pert=pert_depth*random.randint(-10,11)/20
		(x,y,z)=pointlist[i]
		if norms!=None:
			x+=normlist[i][0]*pert
			y+=normlist[i][1]*pert
			z+=normlist[i][2]*pert
		else:
			z+=pert
		pointlist[i]=(x,y,z)
	
	u_c=[min(color[0]+pert_color,255),min(color[1]+pert_color,255),min(color[2]+pert_color,255)]
	l_c=[max(color[0]-pert_color,0),max(color[1]-pert_color,0),max(color[2]-pert_color,0)]
	i=0
	for point in pointlist:
		if random.randint(1,9)==1: # 1/10 
			pert=random.randint(-10,11)
			color=[max(min(color[0]+pert,u_c[0]),l_c[0]),max(min(color[1]+pert,u_c[1]),l_c[1]),max(min(color[2]+pert,u_c[2]),l_c[2])]
		color_value=" "+str(color[0])+" "+str(color[1])+" "+str(color[2])+" 255"
		
		norm_value=""
		if norms!=None:
			norm_value=" "+str(normlist[i][0])+" "+str(normlist[i][1])+" "+str(normlist[i][2])
		datafile.write(list_to_string(point)+norm_value+color_value+supp_value+"\n")
		i+=1
	for face in facelist:
		datafile.write("3 "+list_to_string(face)+"\n")
	datafile.close()
	
	return


def make_cube(width,depth,height,dx, mesh_name,color):
	print("\tCreate a cube {} of dimensions ({},{},{}) with precision {}...".format(mesh_name,str(width),str(depth),str(height),str(dx)))
	n_x=int(width/dx)
	n_y=int(depth/dx)
	n_z=int(height/dx)
	
	i=1
	faces=[]
	norms=[]
	norm_p=[0,0,1]
	norm_n=[0,0,-1]
	n=[n_x,n_y,n_z]
	for p in [(0,1,2),(1,2,0),(2,0,1)]:
		side=(n[p[0]],n[p[1]],n[p[2]])
		for sign in [-1,1]:
			print("\t\tGenerate face n°{}".format(str(i)))
			i=i+1
			z=sign*(dx*side[0])/2
			face=[]
			for  a in range(side[1]+1):
				y=(a-(side[1]/2))*dx
				line=[]
				for b in range(side[2]+1):
					x=(b-(side[2]/2))*dx
					#pert=pert_depth*random.randint(-10,11)/20
					us=(x,y,z)
					line.append((us[p[0]],us[p[1]],us[p[2]]))
					
				face.append(line)
			for l in range(len(face)-1):
				for c in range(len(face[0])-1):
					if sign==1:
						faces.append([face[l][c],face[l][c+1],face[l+1][c]])
						faces.append([face[l+1][c+1],face[l+1][c],face[l][c+1]])
						norms.append((norm_p[p[0]],norm_p[p[1]],norm_p[p[2]]))
						norms.append((norm_p[p[0]],norm_p[p[1]],norm_p[p[2]]))
					else:
						faces.append([face[l][c],face[l+1][c],face[l][c+1]])
						faces.append([face[l+1][c+1],face[l][c+1],face[l+1][c]])
						norms.append((norm_n[p[0]],norm_n[p[1]],norm_n[p[2]]))
						norms.append((norm_n[p[0]],norm_n[p[1]],norm_n[p[2]]))
	datalist_to_ply(faces,mesh_name,color,norms)
	print("\t\tDone")	
	return 


def make_cubes(number,dx,x_l,x_u,y_l,y_u,z_l,z_u,name="data/originaldata/cube"):
	for a in range(number):
		color=[random.randint(0,256),random.randint(0,256),random.randint(0,256)]
		x=random.randint(int(x_l/dx),int(x_u/dx)+1)*dx
		y=random.randint(int(y_l/dx),int(y_u/dx)+1)*dx
		z=random.randint(int(z_l/dx),int(z_u/dx)+1)*dx
		make_cube(x,y,z,dx,name+str(a+1),color)
		
		ply_to_obj(name+str(a+1))
		#ply_to_stl(name+str(a+1))

###############################################################################################

make_cubes(10,dx,0.1,0.8,0.1,0.8,0.1,0.8)

