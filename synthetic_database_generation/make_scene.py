import open3d as o3d
import os
import sys
import time

import pybullet as p
import pybullet_data

import cv2

import math
import numpy as np
import matplotlib.pyplot as plt
import random #random.randint(start, stop+1)

import pathlib

import pymeshlab

import copy


############################################################################################

def add_note(note,path="data/cleandata/note.txt",cleanfilefirst=True):
	if cleanfilefirst:
		f=open(path,'w+')
		f.truncate()
	else:
		f=open(path,'a+')
	f.write(note)
	f.close()
	return


def roundlist(l,n):
	for i in range(len(l)):
		l=list(l)
		if type(l[i])==list or type(l[i])==tuple:
			
			l[i]=roundlist(l[i],n)
		else:
			l[i]=round(l[i],n)
		l=tuple(l)
	return l


def make_stl(ply_names):
	stl_names=[]
	for name in ply_names:
		ms = pymeshlab.MeshSet()
		ms.load_new_mesh(name)
		ms.compute_normals_for_point_sets()
		ms.surface_reconstruction_screened_poisson()
		ms.compute_color_transfer_vertex_to_face()
		#ms.meshing_invert_face_orientation()
		ms.save_current_mesh(name[0:-4]+".stl")	
		stl_names.append(name[0:-4]+".stl")
	return stl_names
	
def make_obj(ply_names):
	obj_names=[]
	for name in ply_names:
		ms = pymeshlab.MeshSet()
		ms.load_new_mesh(name)
		ms.compute_normals_for_point_sets()
		ms.surface_reconstruction_screened_poisson()
		ms.compute_color_transfer_vertex_to_face()
		
		ms.compute_texcoord_by_function_per_vertex()
		ms.compute_texcoord_transfer_vertex_to_wedge()
		ms.compute_texcoord_parametrization_triangle_trivial_per_wedge()
		
		ms.compute_color_by_function_per_face()#define material in .mtl file (ref: https://people.sc.fsu.edu/~jburkardt/data/mtl/mtl.html)
		
		ms.compute_texmap_from_color(textname=f""+name[-5:-4])#define color map in .png file
		ms.save_current_mesh(name[0:-4]+".obj")
		obj_names.append(name[0:-4]+".obj")
	return obj_names

def make_label_txt(seg_opengl,path,class_names=[]):
	float_array=seg_opengl*500
	seg_opengl= float_array.astype(int)
	k=np.unique(seg_opengl)
	size=seg_opengl.shape
	text=''
	image=np.zeros(size)
	for t in range(len(k)-1):
		if class_names !=[]:
			text+=str(class_names[t])+':'
		for a in range(len(seg_opengl)):
			for b in range(len(seg_opengl[0])):
				if seg_opengl[a][b]==k[t+1]:
					text+=str(a)+' '+str(b)+' '
		text+='\n'
	f=open(path,'w+')
	f.write(text)
	f.close()
	return

def make_label_1pic(seg_opengl):
	float_array=seg_opengl*500
	seg_opengl= float_array.astype(int)
	k=np.unique(seg_opengl)
	size=seg_opengl.shape
	image=np.zeros(size)
	for t in range(len(k)-1):
		for a in range(len(seg_opengl)):
			for b in range(len(seg_opengl[0])):
				if seg_opengl[a][b]==k[t+1]:
					image[a][b]=255-(1+t)*20
	return image

def make_label(seg_opengl,path):
	float_array=seg_opengl*500
	seg_opengl= float_array.astype(int)
	k=np.unique(seg_opengl)
	size=seg_opengl.shape
	
	for t in range(len(k)-1):
		image=np.zeros(size)
		for a in range(len(seg_opengl)):
			for b in range(len(seg_opengl[0])):
				if seg_opengl[a][b]==k[t+1]:
					image[a][b]=255#np.asarray([255,255,255])
					
		cv2.imwrite(path+"_"+str(t)+".jpg",image)
	return





def position_closeby(objectIDs):
	# OBJECT PLACEMENT
	Lx=0
	Ly=0
	lx=0
	ly=0
	dir_pos=[[[-1,0],[1,0]],[[0,-1],[0,1]]]
	dir_name=[["at left"," at right"],["on the back"," in front"]]
	dir_try=False
	
	dist=0
	nogo=[]
	dir_prev=[0,0]
	h_or_v=0
	p_or_n=0
	print("\tPlace element n°1 at center")
	for elem in objectIDs:
		aabb = p.getAABB(elem)
		aabbMin = aabb[0]
		aabbMax = aabb[1]
		
		lx_prev=lx
		ly_prev=ly
		lx=aabbMax[0]-aabbMin[0]
		ly=aabbMax[1]-aabbMin[1]
		lz=aabbMax[2]-aabbMin[2]
		
		Lx_save=Lx
		Ly_save=Ly
		Lz=lz/2+dist
		
		while (dir_try):
			#print(nogo)
			while dir_pos[h_or_v][p_or_n] in nogo or dir_pos[h_or_v][p_or_n]==dir_prev:
				h_or_v=random.randint(0, 2) #0=left-right|1=front-back
				if h_or_v==2:
					h_or_v=1 #prefer left & right=> more space
				p_or_n=random.randint(0, 1) #0=left-back |1=right-front
			dir_chosen=dir_pos[h_or_v][p_or_n]
			dir_prev=dir_chosen
			
			print("\tPlace element n°{} {}".format(elem,dir_name[h_or_v][p_or_n]))
			
			Lx+=(lx_prev+lx)/2*dir_chosen[0]
			Ly+=(ly_prev+ly)/2*dir_chosen[1]
			
			Lx+=dist*dir_chosen[0]
			Ly+=dist*dir_chosen[1]
			
			#print("\tLx {}".format(Lx))
			#print("\tLy {}".format(Ly))
			
			aabbMin_n=[Lx-lx/2,Ly-ly/2,0]
			aabbMax_n=[Lx+lx/2,Ly+ly/2,lz]
			#print(p.getOverlappingObjects(aabbMin_n,aabbMax_n))
			if len(p.getOverlappingObjects(aabbMin_n,aabbMax_n))>2:
				Lx=Lx_save
				Ly=Ly_save
				#dir_try=False
				nogo.append(dir_chosen)
				print("\t\tOverlapping !")
			else:
				print("\t\tOK !")
				nogo=[]#[dir_chosen]
				dir_try=False
			if len(nogo)==4:
				dist+=0.01
				print("\t\tNo possible directions. Re-try with distance of {}".format(dist))
				nogo=[]
			#time.sleep(1)
		dir_try=True
		dist=0
		
		p.resetBasePositionAndOrientation(elem,[Lx,Ly,Lz],[0,0,0,1])
	return





def generate_random_scene(names, image_res_name="example",saving_images=False,class_names=[],scene_type="random"):
	
	### SIMULATION PARAMETERS:
	cid= p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')

	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	p.setPhysicsEngineParameter(numSolverIterations=10)
	p.setTimeStep(1. / 120.)
	logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")
	#useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)
	p.loadURDF("plane_transparent.urdf", useMaximalCoordinates=True)
	
	
	#enable rendering during creation.
	p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
	p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
	
	#disable tinyrenderer, software (CPU) renderer, we don't use it here
	p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
	
	if saving_images:
		#enable rgb-depth-segmentation images
		p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)
		p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 1)
		p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
	
	
	### CAMERA PARAMETERS:
	width = 1280 #128
	height = 720 #128

	fov = 60
	aspect = width / height
	near = 0.02
	far = 5
	
	view_matrix = p.computeViewMatrix(
	    cameraEyePosition=[0,0,far],
	    cameraTargetPosition=[0, 0, 0],
	    cameraUpVector=[0, 1, 0])	
	projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	
	p.resetDebugVisualizerCamera(cameraDistance=far, cameraYaw=0, cameraPitch=-89, cameraTargetPosition=[0, 0, 0])	
	
	
	
	### OBJECTS SPAWNING:
	#names=make_stl(names)	#if ply files
	models=[]
	positions=[]
	for i in range(len(names)):
		rx=len(names)*random.randint(-10,11)/100
		ry=len(names)*random.randint(-10,11)/100
		position=[rx,ry,1+i/2]
		models.append([names[i],position])
		positions.append([rx,ry,i/2])
	
	shift = [0, -0.02, 0]
	meshScale = [1, 1, 1]
	objectIDs=[]
	for model in models:
		color=[random.randint(-10,11)/10,random.randint(-10,11)/10,random.randint(-10,11)/10,1]
		visualShapeId=p.createVisualShape(shapeType=p.GEOM_MESH,
				                    fileName=model[0],
				                    #rgbaColor=color,
				                    #specularColor=[0.4, .4, 0],
				                    visualFramePosition=shift,
				                    meshScale=meshScale)
		collisionShapeId=p.createCollisionShape(shapeType=p.GEOM_MESH,
				                          fileName=model[0],
				                          collisionFramePosition=shift,
				                          meshScale=meshScale)
		a=p.createMultiBody(baseMass=1.0,
		              baseInertialFramePosition=[0, 0, 0],
		              baseCollisionShapeIndex=collisionShapeId,
		              baseVisualShapeIndex=visualShapeId,
		              basePosition=model[1],
		              useMaximalCoordinates=True)
		objectIDs.append(a)

	#for elem in objectIDs:
		#print("object ID {} is at pose :".format(elem)+ str(p.getBasePositionAndOrientation(elem)))
	
	
	### START SIMULATION
	p.stopStateLogging(logId)
	p.setGravity(0, 0, -10)
	p.setRealTimeSimulation(1)
	
	### RESET OBJECT POSITION (if scene type different that random)
	if scene_type=="closeby":
		position_closeby(objectIDs)
	
	### WAIT FOR STABLE POSITION
	start_time = time.time()
	unstable=True
	while unstable:
		if time.time()-start_time>10: #10sec execution time MAX
			break
		time.sleep(1./60.)
		unstable=False
		i=0
		for elem in objectIDs:
	     		#print("object ID {} is at pose :".format(elem)+ str(roundlist(p.getBasePositionAndOrientation(elem),4)))
	     		if p.getBasePositionAndOrientation(elem)[0][2]<0:
	     			print("object ID {} has passed through the floor ! It was respawn.".format(elem))
	     			rx=random.randint(-10,11)/10
	     			ry=random.randint(-10,11)/10
	     			rz=random.randint(-20,21)/10
	     			p.resetBasePositionAndOrientation(elem,[rx,ry,rz],[0,0,0,1])
	     			unstable=True
	     		if positions[i]!=roundlist(p.getBasePositionAndOrientation(elem),4):
	     			unstable=True
	     		positions[i]=roundlist(p.getBasePositionAndOrientation(elem),4)
	     		i=i+1
	
	
	
	# SAVE IMAGES
	if saving_images and not unstable:
		images = p.getCameraImage(width,
		                  height,
		                  view_matrix,
		                  projection_matrix,
		                  shadow=True,
		                  renderer=p.ER_BULLET_HARDWARE_OPENGL)
		# NOTE: the ordering of height and width change based on the conversion
		rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
		depth_buffer_opengl = np.reshape(images[3], [height, width])
		depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
		seg_opengl = np.reshape(images[4], [height, width]) * 1. / 255.
		
		#change background to white
		lab=np.array(seg_opengl, dtype=bool)
		a=rgb_opengl[0][0][3]
		for i in range(len(seg_opengl.tolist())):
			for j in  range(len(seg_opengl.tolist()[0])):
				if not lab[i][j]:
					#print(rgb_opengl[i][j])
					rgb_opengl[i][j]=[1,1,1,a]
		
		#normalize depth
		b=np.unique(depth_opengl).tolist()
		b.remove(max(b))
		a=max(b)
		for i in range(len(depth_opengl.tolist())):
			for j in range(len(depth_opengl.tolist()[0])):
				if depth_opengl[i][j]==1:
					depth_opengl[i][j]=a
		print(depth_opengl.min())
		print(depth_opengl.max())
		normalized_image = depth_opengl
		normalized_image=(depth_opengl - depth_opengl.min()) / (depth_opengl.max() - depth_opengl.min())
		
		
		#save rgb & depth images in .jpg
		#plt.imsave("data/images/depth/"+image_res_name+".jpg", depth_opengl)
		plt.imsave("data/images/depth/"+image_res_name+".jpg", normalized_image)
		plt.imsave("data/images/rgb/"+image_res_name+".jpg", rgb_opengl)
		
		#save label in .txt
		make_label_txt(seg_opengl,"data/images/seg/"+image_res_name+".txt",class_names=class_names)
		
		
		#wait for finishing saving to finish before next simulation
		for a in ["depth/"+image_res_name+".jpg","rgb/"+image_res_name+".jpg","seg/"+image_res_name+".txt"]:
			while not pathlib.Path("data/images/"+a).is_file():
				time.sleep(1./60.)
		
	p.disconnect()
	
	if unstable: #recursion if unstable => relaunch the function!
		positions=generate_random_scene(names, image_res_name,saving_images,class_names,scene_type)
	
	return positions




def points_label_redorgreen(pcd,label):
	c=[[1, 0, 0],[0,1,0]] #0=red 1=green
	a=[]
	for v in range(2):
		out_pc = o3d.geometry.PointCloud()
		out_pc.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[label == v])
		out_pc.paint_uniform_color(c[v])
		a.append(out_pc)
	#o3d.visualization.draw_geometries(a)
	return a

def points_label_colors(pcd,label):
	c=[[1, 0, 0],[0,1,0],[0, 0, 1],[1,1,0],[0,1,1],[1,0,1],[1,1,1]]
	a=[]
	prev=0
	for v in range(len(label)):
		if v >=7:
			c.append([random.uniform(0,1),random.uniform(0,1),random.uniform(0,1)])
		out_pc = o3d.geometry.PointCloud()
		out_pc.points = o3d.utility.Vector3dVector(np.asarray(pcd.points[prev:label[v]-1]))
		out_pc.paint_uniform_color(c[v])
		a.append(out_pc)
		prev=label[v]
	#o3d.visualization.draw_geometries(a)
	return a

def points_label_i(pcd,label,i=1):
	out_pcd = o3d.geometry.PointCloud()
	out_pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[label == i])
	out_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[label == i])
	return out_pcd

def merge_labels(labels,label):
	merged_label=[]
	old=0
	old2=0
	for i in range(len(labels)):
		l=label[old:labels[i]]
		merged_label.append(np.count_nonzero(l == 1)+old2)
		old=labels[i]
		old2=merged_label[i]
	return merged_label


def select_points_viewed_by_camplan(pcd,cam_pose,precision=0.9):
	#A=cam_plan[0];B=cam_plan[1];C=cam_plan[2];D=cam_plan[3]
	#CP=math.sqrt(A**2+B**2+C**2)
	points=np.asarray(pcd.points)
	label=np.zeros(len(points)).tolist()
	points_OK=[]
	for a in range(len(points)):
		#print("{}/{} points".format(a,len(points)))
		condition=True
		for b in range(len(points_OK)):
			cp=[points[a][0],points[a][1],10]
			dist_a= np.array(points[a])-np.array(cp)
			dist_b= np.array(points[points_OK[b]])-np.array(points[a])
			dist_c= np.array(points[points_OK[b]])-np.array(cp)
			unit_vector_1 = dist_a / np.linalg.norm(dist_a)
			unit_vector_2 = dist_b / np.linalg.norm(dist_b)
			dot_product = np.dot(unit_vector_1, unit_vector_2)
			if dot_product>0 and dot_product>precision:
				if np.linalg.norm(dist_a) < np.linalg.norm(dist_c): #if point is closer to CAM than ref point
					condition=False
					points_OK[b]=a
		if condition:
			points_OK.append(a)
		
	for i in points_OK:
		label[i]=1
	label=np.asarray(label)
	return label
	
def select_points_viewed_by_cam(pcd,cam_pose,precision=0.9):
	points=np.asarray(pcd.points)
	label=np.zeros(len(points)).tolist()
	points_OK=[]
	for a in range(len(points)):
		#print("{}/{} points".format(a,len(points)))
		condition=True
		for b in range(len(points_OK)):
			dist_a= np.array(points[a])-np.array(cam_pose)
			dist_b= np.array(points[points_OK[b]])-np.array(points[a])
			dist_c= np.array(points[points_OK[b]])-np.array(cam_pose)
			unit_vector_1 = dist_a / np.linalg.norm(dist_a)
			unit_vector_2 = dist_b / np.linalg.norm(dist_b)
			dot_product = np.dot(unit_vector_1, unit_vector_2)
			if dot_product>0 and dot_product>precision:
				if np.linalg.norm(dist_a) < np.linalg.norm(dist_c): #if point is closer to CAM than ref point
					condition=False
					points_OK[b]=a
		if condition:
			points_OK.append(a)
		
	for i in points_OK:
		label[i]=1
	label=np.asarray(label)
	return label

def get_label_matrix(labels):
	label_matrix=[]
	old=0
	for i in range(len(labels)):
		l=i*np.ones(labels[i]-old)
		l = [int(x) for x in l.tolist()] 
		old=labels[i]
		label_matrix.extend(l)
	return label_matrix

def adapt_dataset(ply_names,positions,cam_pose=[0,0,10],datasetname="example",cleandatafirst=True):
	#http://www.open3d.org/docs/0.10.0/tutorial/Basic/transformation.html
	#http://www.open3d.org/html/tutorial/Basic/transformation.html
	meshes=[]
	meshes_label=[]
	meshes_camview=o3d.geometry.PointCloud()
	labels=[]
	for i in range(len(ply_names)):
		name=ply_names[i]
		T=positions[i][0] # (1.3,0,0)
		#R_quat=positions[i][1]
		R_euler=p.getEulerFromQuaternion(positions[i][1])
		#R_angles=p.getAxisAngleFromQuaternion(positions[i][1])
		mesh = o3d.io.read_point_cloud(name)
		#mesh_rotated=copy.deepcopy(mesh).rotate(mesh.get_rotation_matrix_from_quaternion(R_quat), center=mesh.get_center())
		#mesh_rotated=copy.deepcopy(mesh).rotate(mesh.get_rotation_matrix_from_axis_angle(R_angles), center=mesh.get_center())
		mesh_rotated=copy.deepcopy(mesh).rotate(mesh.get_rotation_matrix_from_xyz(R_euler), center=mesh.get_center())
		mesh_translated = copy.deepcopy(mesh_rotated).translate(T,relative=True)
		#o3d.visualization.draw_geometries([mesh, mesh_translated,mesh_rotated])
		meshes.append(mesh_translated)
		
		pcd=mesh_translated
		pcd = pcd.voxel_down_sample(voxel_size=0.05)
		
		label=select_points_viewed_by_cam(pcd,cam_pose,0.7)
		
		meshes_label.extend(points_label_redorgreen(pcd,label))
		meshes_camview += points_label_i(pcd,label)
		labels.append(len(meshes_camview.points))
	
	
	print(labels)
	print(len(meshes_camview.points))
	label=select_points_viewed_by_cam(meshes_camview,cam_pose,0.95)
	o3d.visualization.draw_geometries(points_label_redorgreen(meshes_camview,label))
	meshes_camview=points_label_i(meshes_camview,label)
	labels=merge_labels(labels,label)
	print(labels)
	print(len(meshes_camview.points))
	
	o3d.visualization.draw_geometries(meshes)		
	o3d.visualization.draw_geometries(meshes_label)
	o3d.visualization.draw_geometries([meshes_camview])

	o3d.io.write_point_cloud("data/cleandata/"+datasetname+".ply", meshes_camview, write_ascii=False, compressed=False, print_progress=True)
	o3d.visualization.draw_geometries(points_label_colors(meshes_camview,labels))
	
	add_note(datasetname+".ply"+" "+str(labels)+"\n",cleanfilefirst=cleandatafirst)
	return
	

def random_set(names,max=None,classes=None,dif_obj=True):
	if max is None:
		max=len(names)
	number_objects=random.randint(1,max)
	choices=[i for i in range(len(names))]
	selected_names=[]
	selected_classes=[]
	if dif_obj:
		for a in range(number_objects):
			#print(choices)
			choice=random.randint(0,len(choices)-1)
			#print(choice)
			selected_names.append(names[choices[choice]])
			if classes!=None:
				selected_classes.append(classes[choices[choice]])
			del choices[choice]
	else:
		choice=random.randint(0,len(choices)-1)
		selected_names=[names[choices[choice]]] * number_objects
		if classes!=None:
			selected_classes= [classes[choices[choice]]]*number_objects
	
	return (selected_names, selected_classes)

############################################################################################
print("Generate Synthetic Dataset...")


print("*Choosing MODELS DATASET to use:")
n_cube = int(input("\t How many CUBES in data folder ? [Default is 60]\n\t-> "))
n_cylinder = int(input("\t How many CYLINDERS in data folder ? [Default is 60]\n\t-> "))

sim_or_dif = int(input("\t Similar or different objects in the scene ? [Default is 1]\n\t\t[0] SAME-> type '0'\n\t\t[1] DIFFERENT-> type '1'\n\t-> "))

#dataset reference
obj_names=[]
class_names=[]

for a in range(n_cube):
	obj_names.append("data/originaldata/cube{}.obj".format(a+1))
	class_names.append(0)
for a in range(n_cylinder):
	obj_names.append("data/originaldata/cylinder{}.obj".format(a+1))
	class_names.append(1)

print("*Choosing SYNTHETIC DATA to generate:")
n_scene=int(input("\t How many ? [Default is 100]\n\t-> "))
init=int(input("\t Starting at... ? [Default is 0]\n\t-> "))
max_obj=int(input("\t Maximum number of objects per scene ? [Default is 10]\n\t-> "))
t_scene=int(input("\t Type of the scene ? [Default is 1]\n\t\t[0] clustered objects next to each other-> type '0'\n\t\t[1] objects randomly clustered-> type '1'\n\t-> "))

scene_types=["closeby","random"]

print("*Start SYNTHETIC DATASET GENERATION:")
for z in range(n_scene):
	(names,classes)=random_set(obj_names,max=max_obj,classes=class_names,dif_obj=sim_or_dif)
	print((names,classes))
	positions=generate_random_scene(names,str(z+init),saving_images=True,class_names=classes,scene_type=scene_types[t_scene])

