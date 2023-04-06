import open3d as o3d
import numpy as np
import math
import os
import sys
import random #random.randint(start, stop+1)
import copy

import pymeshlab

#COMMON PARAMETERS
#dx=0.01
#pert_depth=dx/2
#pert_color=100


#######################################################################################

def ply_to_stl(ply_name):
	print("\tWrite on the stl file '{}'".format(ply_name+".stl"))
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
	print("\tWrite on the obj file '{}'".format(ply_name+".obj"))
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
	
	ms.compute_texmap_from_color(textname=f""+"texture/"+ply_name.split("/")[-1]+".png")#define color map in .png file
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

def datalist_to_ply(datalist,mesh_name,color,dx,norms=None,pert_depth=0,pert_color=0,noise_type=None):
	print("\tWrite on the ply file '{}'".format(mesh_name+".ply"))
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
	
	
	print("\t\tAdd spacial disturbances ?",end='')
	#choose a certain number of disturbances (bumps like)
	if noise_type==None:
		print("\tNo")
	elif noise_type=="realistic":
		print("\trealistic noise (=bumps similar to camera depth information)")
		pert_d=[]
		n_pert=len(pointlist)//1000
		for a in range(n_pert):
			pert_d.append([random.randint(0,len(pointlist)+1),1])
		
		pert=0
		for i in range(len(pointlist)):
			(x,y,z)=pointlist[i]
			
			#see if disturbances (bumps like)
			for d in pert_d:
				(xd,yd,zd)=pointlist[d[0]]
				a=np.subtract(np.array([x,y,z]),np.array([xd,yd,zd]))
				if np.linalg.norm(a)<10*dx:#close to a declencher
					if d[1]:
						pert+=pert_depth
						d[1]=0
						#print('True')
				else:
					if not d[1]:
						d[1]=1
						pert=0
			
			#add disturbances (bumps like)
			if norms!=None:
				x+=normlist[i][0]*pert
				y+=normlist[i][1]*pert
				z+=normlist[i][2]*pert
			else:
				z+=pert
			pointlist[i]=(x,y,z)
			
	elif noise_type=="random":
		print("\trandom noise")
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
	
	
	print("\t\tAdd color disturbances")
	pert=0
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


def make_cylinder(radius,height,dx, mesh_name,color,pert_depth=0,pert_color=0,noise_type=None):
	print("\tCreate a cylinder {} of dimensions ({},{}) with precision {}...".format(mesh_name,str(radius),str(height),str(dx)))
	n_r=int(radius/dx)
	n_z=int(height/dx)
	n_c=5*(2**(n_r-1))
	
	name=["bottom","side","top"]
	faces=[]
	norms=[]
	side_circle=[]
	k=2
	#make cylinder top & bottom points
	for t_or_b in [-1,1]:
		print("\t\tGenerate cylinder {} face".format(name[t_or_b+1]))
		z=t_or_b*height/2
		circles=[]
		for c in range(n_r):
			circle=[]
			radius_c=radius*(n_r-c)/n_r
			for p in range(int(n_c/(2**c))):
				x=radius_c*math.cos(2*math.pi*p/int(n_c/(2**c)))
				y=radius_c*math.sin(2*math.pi*p/int(n_c/(2**c)))
				circle.append((x,y,z))
			circles.append(circle)
		for ring in range(n_r-1):
			circle_out=circles[ring]
			circle_in=circles[ring+1]
			for i in range(len(circle_in)):
				if t_or_b==1:
					faces.append([circle_in[(i-k)],circle_out[2*(i-k)],circle_in[i-k+1]])
					faces.append([circle_out[2*(i-k)],circle_out[2*(i-k)+1],circle_in[i-k+1]])
					faces.append([circle_out[2*(i-k)+1],circle_out[2*(i-k)+2],circle_in[i-k+1]])
				else:
					faces.append([circle_in[(i-k)],circle_in[i-k+1],circle_out[2*(i-k)]])
					faces.append([circle_out[2*(i-k)],circle_in[i-k+1],circle_out[2*(i-k)+1]])
					faces.append([circle_out[2*(i-k)+1],circle_in[i-k+1],circle_out[2*(i-k)+2]])
				norms.append((0,0,-t_or_b))
				norms.append((0,0,-t_or_b))
				norms.append((0,0,-t_or_b))
		for i in range(len(circles[-1])):
			if t_or_b==1:
				faces.append([circles[-1][i-1-k],circles[-1][i-k],(0,0,z)])
			else:
				faces.append([circles[-1][i-1-k],(0,0,z),circles[-1][i-k]])
			norms.append((0,0,t_or_b))
		side_circle.append(circles[0])
	
	#make cylinder edges
	print("\t\tGenerate cylinder {} face".format(name[1]))
	circles=[]
	for c in range(n_z-1):
		z=height*((n_z-c)/n_z)-height/2
		circle=[]
		for p in range(len(side_circle[1])):
			x,y,z_old=side_circle[1][p]
			circle.append((x,y,z))
		circles.append(circle)
	circles.append(side_circle[0])
	for ring in range(len(circles)-1):
		circle_top=circles[ring]
		circle_bottom=circles[ring+1]
		for i in range(len(circle_top)):
			x,y,z=circle_top[i-k]
			faces.append([circle_top[i-k],circle_bottom[i-k],circle_top[i+1-k]])
			norm=(math.acos(x/radius)/math.pi,math.asin(y/radius)/math.pi,0)
			norms.append(norm)
			faces.append([circle_top[i+1-k],circle_bottom[i-k],circle_bottom[i+1-k]])
			norms.append(norm)
	datalist_to_ply(faces,mesh_name,color,dx,norms,pert_depth,pert_color,noise_type)
	
	print("\t\tDone")	
	return 


def make_cube(width,depth,height,dx, mesh_name,color,pert_depth=0,pert_color=0,noise_type=None):
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
	datalist_to_ply(faces,mesh_name,color,dx,norms,pert_depth,pert_color,noise_type)
	print("\t\tDone")	
	return 


def make_cubes(number,dx,x_l,x_u,y_l,y_u,z_l,z_u,name="data/originaldata/cube",init=0,pert_depth=0,pert_color=0,noise_type=None):
	for a in range(number):
		color=[random.randint(0,256),random.randint(0,256),random.randint(0,256)]
		x=round(random.randint(int(x_l/dx),int(x_u/dx)+1)*dx,5)
		y=round(random.randint(int(y_l/dx),int(y_u/dx)+1)*dx,5)
		z=round(random.randint(int(z_l/dx),int(z_u/dx)+1)*dx,5)
		make_cube(x,y,z,dx,name+str(a+1+init),color,pert_depth,pert_color,noise_type)
		
		ply_to_obj(name+str(a+1+init))
		#ply_to_stl(name+str(a+1))

def make_cylinders(number,dx,r_l,r_u,z_l,z_u,name="data/originaldata/cylinder",init=0,pert_depth=0,pert_color=0,noise_type=None):
	for a in range(number):
		color=[random.randint(0,256),random.randint(0,256),random.randint(0,256)]
		r=x = round(random.randint(int(r_l/dx),int(r_u/dx)+1)*dx,5)
		z=round(random.randint(int(z_l/dx),int(z_u/dx)+1)*dx,5)
		make_cylinder(r,z,dx,name+str(a+1+init),color,pert_depth,pert_color,noise_type)
		
		ply_to_obj(name+str(a+1+init))
		#ply_to_stl(name+str(a+1))

###############################################################################################
print("Generate Shapes...")

bool_cube = str(input("\n* Make CUBES ?\nyes -> type 'yes'\nno -> type 'ENTER'\n-> "))

if bool_cube=="yes":
	n_cube=int(input('\tHow many ?\n\t-> '))
	i_cube=int(input('\tStarting at ... ? [Default is 0]\n\t-> '))
	dx_cube=float(input('\tPrecision?[Default is 0.01]\n\t-> '))

bool_cylinder =str(input("\n*Make CYLINDERS ?\nyes -> type 'yes'\nno -> type 'ENTER'\n-> "))
if bool_cylinder=="yes":
	n_cylinder=int(input('\tHow many ?\n\t-> '))
	i_cylinder=int(input('\tStarting at ... ? [Default is 0]\n\t-> '))
	dx_cylinder=float(input('\tPrecision?[Default is 0.025]\n\t-> '))

print("\n* Add NOISE ?\n")
noise_type=int(input("\tTYPE of noise?[Default is 1]\n\t[0] NO noise-> type '0'\n\t[1] realistic noise-> type '1'\n\t[2] random noise -> type '2'\n\t-> "))
if noise_type in [1,2] :
	pert_depth=float(input('\t\tSPACIAL noise?[Default is 0.005]\n\t\t-> '))
	pert_color=int(input('\t\tCOLOR noise?[Default is 10]\n\t\t-> '))
else:
	noise_type=0
	pert_depth=0
	pert_color=0

noise_types=[None,"realistic","random"]

if bool_cube=="yes":
	make_cubes(n_cube,dx_cube,0.1,0.8,0.1,0.8,0.1,0.8,init=i_cube,pert_depth=pert_depth,pert_color=pert_color,noise_type=noise_types[noise_type])
if bool_cylinder=="yes":
	make_cylinders(n_cylinder,dx_cylinder,0.05,0.2,0.1,0.8,init=i_cylinder,pert_depth=pert_depth,pert_color=pert_color,noise_type=noise_types[noise_type])

#make_cubes(60,dx,0.1,0.8,0.1,0.8,0.1,0.8,init=0)
#make_cylinders(60,dx,0.05,0.2,0.1,0.8,init=0)



