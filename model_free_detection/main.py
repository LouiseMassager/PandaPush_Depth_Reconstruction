import cv2
import pyrealsense2 as rs
from realsense_camera import *

#import tensorflow as tf
import open3d as o3d
import pymeshlab

import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
import copy

from sklearn.cluster import *
from sklearn.preprocessing import StandardScaler

from framechange import *

import pybullet as p
import pybullet_data

############################################################################
######################## DATA ACQUISITION (.bag to .ply/.png)
############################################################################

def choose_datasource(): #PLAYBACK (.bag) OR REAL TIME STREAM
	try:
		source = "data/recordings/"+str(sys.argv[1]) #ex: "outdoors.bag"
		print("use recording :'"+source+"'")
	except:
		source=None
		print("real time stream")
	return source

def tensorflow_model_loading(): #DETECTION with tensorflow (! only rgb => 2D!)
	with tf.compat.v1.gfile.FastGFile('frozen_inference_graph.pb', 'rb') as f: 
		graph_def = tf.compat.v1.GraphDef()
		graph_def.ParseFromString(f.read())
		
	def detection(img):
		# Restore session
		sess.graph.as_default()
		tf.import_graph_def(graph_def, name='')
		# Read and preprocess an image.
		rows = img.shape[0]
		cols = img.shape[1]
		inp = cv2.resize(img, (300, 300))
		inp = inp[:, :, [2, 1, 0]]  # BGR2RGB
		# Run the model
		out = sess.run([sess.graph.get_tensor_by_name('num_detections:0'),
                        sess.graph.get_tensor_by_name('detection_scores:0'),
                        sess.graph.get_tensor_by_name('detection_boxes:0'),
                        sess.graph.get_tensor_by_name('detection_classes:0')],
                       feed_dict={'image_tensor:0': inp.reshape(1, inp.shape[0], inp.shape[1], 3)})
                       
		# Visualize detected bounding boxes.
		num_detections = int(out[0][0])
		for i in range(num_detections):
			classId = int(out[3][0][i])
			score = float(out[1][0][i])
			bbox = [float(v) for v in out[2][0][i]]
			if score > 0.3:
				x = bbox[1] * cols
				y = bbox[0] * rows
				right = bbox[3] * cols
				bottom = bbox[2] * rows
				cv2.rectangle(img, (int(x), int(y)), (int(right), int(bottom)), (125, 255, 51), thickness=2)
	return

def data_aquisition(detection=False,saving=True,showing=False,frame_numbers=[4]):
	""" PARAMETERS:
	detection=False 	#false if don't want to detect with tensorflow
	saving = True		#false if don't want to save .png and .ply format of depth/rgb
	showing = False	#false if don't want windows with depth and color stream to pop-up
	"""
	#1. choose source of data
	source=choose_datasource()
	
	#2. load ML model
	if detection:
		import tensorflow as tf
		tensorflow_model_loading()
	
	#3. define camera parameters
	cam = RealsenseCamera(recording=source)
	
	#4. Create loop to read frames continously
	i=0;imax=len(frame_numbers)
	while i<imax: #at default, only take 5th frame
		fn=str(frame_numbers[i])
		
		#Read Bgr and Depth Frame in real time from Realsense Camera
		ret, bgr_frame, depth_frame = cam.get_frame_stream()
		
		#if there is a depth and color frame
		if ret:
			if saving:
				cam.saveframe("color/"+fn,bgr_frame)
				cam.saveframe("depth/"+fn,depth_frame)
				cam.saveply(fn,bgr_frame, depth_frame)
		else:
			break
		
		#detection with tensorflow
		if detection:
			with tf.compat.v1.Session() as sess:
				detection(bgr_frame)
		#display the depth frame & bgr frame(Color Frame)
		if showing:
			cv2.imshow("depth frame", depth_frame)
			cv2.imshow("bgr frame", bgr_frame)
			
		# Quit the loop if we press escape key on keyboard
		key = cv2.waitKey(500)
		if key == 27:
			break
		i+=1
	return


############################################################################
######################## SEGMENTATION (.ply)
############################################################################
def remove_board_outliers(pcd):
	#ref: http://www.open3d.org/docs/latest/tutorial/Advanced/pointcloud_outlier_removal.html
	#radius_outlier_removal removes points that have few neighbors in a given sphere around them
	#statistical_outlier_removal removes points that are further away from their neighbors compared to the average for the point cloud.
	print("\tBoard and outliers removal...")
	#note: parameters and methods obtained EMPIRICALLY
	
	print("\t\tstep 0/4")
	loop=True;i=600;initial=len(pcd.points)
	while loop:
		#cl, ind = pcd.remove_radius_outlier(nb_points=100,radius=0.01)
		cl, ind = pcd.remove_radius_outlier(nb_points=1000-i,radius=0.01) #600
		test = pcd.select_by_index(ind)
		i+=10
		if 1000-i<=0:
			loop=False
		elif len(test.points)>initial*0.7: #remove MAX 20% of points
			loop=False
			pcd = test
	
	print("\t\tstep 1/4")
	initial=len(pcd.points)
	cl, ind = pcd.remove_radius_outlier(nb_points=300,radius=0.01) #600
	test = pcd.select_by_index(ind)
	if len(test.points)>initial*0.9: #remove MAX 20% of points
		pcd = test
	
	print("\t\tstep 2/4")
	cl, ind = pcd.remove_statistical_outlier(nb_neighbors=2000,std_ratio=0.001)
	pcd = pcd.select_by_index(ind)
	
	print("\t\tstep 3/4")
	cl, ind = pcd.remove_radius_outlier(nb_points=200,radius=0.01) #600
	pcd = pcd.select_by_index(ind)
	
	print("\t\tstep 4/4")
	return pcd


def background_removal(pcd, threshold,showing=False):
	#ref: https://github.com/isl-org/Open3D/issues/2291
	print("\tBackground removal...")
	print("\t\tremove points under threshold of : {}".format(threshold[3]))
	
	points = np.asarray(pcd.points)
	pcd = pcd.select_by_index(np.where(threshold[0]*points[:,0]+threshold[1]*points[:,1]+threshold[2]*points[:,2] > -threshold[3])[0])

	if showing:
		o3d.visualization.draw_geometries([pcd])
		
	return pcd

def ransac_plane_segmentation(pcd):
	print("\tTabletop detection...")
	threshold=[0,0,0,1000];plane_present=True
	while plane_present:
		plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
		                                 ransac_n=3,
		                                 num_iterations=1000)
		#distance_threshold= max distance point-plane for point to be inlier
		#ransac_n = number of points to estimate a plane
		#num_iterations = how often a random plane is sampled and verified. 

		[a, b, c, d] = plane_model
		inlier_cloud = pcd.select_by_index(inliers)
		outlier_cloud = pcd.select_by_index(inliers, invert=True)
		if len(inlier_cloud.points)>0.5*len(outlier_cloud.points):
			print(f"\t\tPlane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
			if threshold[3]>d:
				threshold=plane_model
				
			pcd=outlier_cloud
		else:
			plane_present=False
			print("\t\tfloor/wall detection finished")
	return pcd,threshold



def dbscan_clustering(pcd,weights=[(2,2,2),(1,1,1)],scaling=None,showing=False):
	print("\tDBSCAN clustering...")
	
	#initialization
	o3d.io.write_point_cloud("data/treatment/step1_Segmentation/total.ply", pcd, write_ascii=False, compressed=False, print_progress=True)
	points=np.asarray(pcd.points)
	colors=np.asarray(pcd.colors)
	vertex=np.asarray(pcd.normals)

	#weights
	x,y,z=weights[0]
	r,g,b=weights[1]
	
	#feature matrix X generation
	X=[];
	for i in range(len(points)): 
		az=np.append(z*points[i][2],r*colors[i][0])
		az=np.append(az,g*colors[i][1])
		az=np.append(az,b*colors[i][2])
		az=np.append(az,x*points[i][0])
		az=np.append(az,y*points[i][1])
		X.append(az)
	
	#SCALING or not
	#ref: https://scikit-learn.org/stable/modules/classes.html#module-sklearn.preprocessing
	if scaling!=None:
		if scaling=="standard":
			###Standardize features by removing the mean and scaling to unit variance:
			scaler = StandardScaler(copy=True,with_mean=True, with_std=True).fit(X) #z = (x - mean) / std
		if scaling=="robust":
			###Scale features using statistics that are robust to outliers:
			scaler= RobustScaler(with_centering=True, with_scaling=True, quantile_range=(25.0, 75.0), copy=True, unit_variance=False).fit(X)
		
		if scaling=="maxabs":
			###Scale each feature by its maximum absolute value:
			scaler=MaxAbsScaler(copy=True).fit(X)
		X = scaler.transform(X)
	
	#DBSCAN clustering (parameters obtained EMPIRICALLY)
	clustering = DBSCAN(eps=0.03, min_samples=300,algorithm='auto',metric='euclidean').fit(X,y=None,sample_weight=None)
	labels=clustering.labels_
	
	#pointcloud separation
	max_label = labels.max()
	print(f"\t\tpoint cloud has {max_label + 1} clusters")
	pcds=[];num_of_objects=max_label+1
	for a in range(num_of_objects):
		out_pc = o3d.geometry.PointCloud()
		out_pc.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[labels == a])
		out_pc.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[labels == a])
		out_pc.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[labels == a])
		pcds.append(out_pc)
		o3d.io.write_point_cloud("data/treatment/step1_Segmentation/"+str(a)+".ply", out_pc, write_ascii=False, compressed=False, print_progress=True)
	
	if showing:	
		o3d.visualization.draw_geometries(pcds)
	print("\tDone")
	
	return num_of_objects

def o3d_segmentation(frame_number=4):
	print("Segmentation...")
	
	#1. open pointcloud
	model="data/ply/"+str(frame_number)+".ply"
	pcd = o3d.io.read_point_cloud(model)
	
	#2. plane segmentation with RANSAC algorithm
	pcd,threshold=ransac_plane_segmentation(pcd)
	
	#3. remove points beyond floor/wall
	pcd=background_removal(pcd,threshold,showing=True)
	
	#4. remove the board and outliers
	pcd=remove_board_outliers(pcd)
	
	#5. DBSCAN clustering
	num_of_objects=dbscan_clustering(pcd,showing=True)
	
	return num_of_objects,threshold

############################################################################
######################## SHAPE COMPLETION (.ply)
############################################################################
def get_object_pose(pcd,threshold):
	points=np.asarray(pcd.points)
	vertex=np.asarray(pcd.normals)
	
	direction =np.mean(vertex,axis=0)
	center =np.mean(points,axis=0)
	
	A=threshold[0];B=threshold[1];C=threshold[2];D=threshold[3]
	x=center[0];y=center[1];z=center[2]
	
	height=abs(A*x+B*y+C*z+D)/sqrt(A**2+B**2+C**2)
	center[2]=center[2]-height/2
	
	print("\tcenter {} and direction {}".format(center,direction))
	pose=[center,direction]
	
	return pose,height

def edge_clustering(pcd,pose):
	points=np.asarray(pcd.points)
	colors=np.asarray(pcd.colors)
	vertex=np.asarray(pcd.normals)

	points= align_to_origin(points,center=pose[0],direction=pose[1])

	sidenormals=[];labels=np.ndarray([])
	for i in range(len(points)): 
		p=points[i]
		neighboors=[]
		q11=False;q12=False;q21=False;q22=False;q31=False;q32=False;q41=False;q42=False;
		for n in points:
			a= np.linalg.norm(np.array(p)-np.array(n))
			if a<0.01 and a!=0:
				neighboors.append(n)
				v=np.subtract(n,p)
				if v[0]<=0:
					if v[1]<=0:
						if abs(v[0])<=abs(v[1]):
							q11=True
						else:
							q12=True
					else:
						if abs(v[0])<=abs(v[1]):
							q21=True
						else:
							q22=True
				else:
					if v[1]<=0:
						if abs(v[0])<=abs(v[1]):
							q31=True
						else:
							q32=True
					else:
						if abs(v[0])<=abs(v[1]):
							q41=True
						else:
							q42=True	
							
		if q11+q12+q21+q22+q31+q32+q41+q42>=7:
			labels=np.append(labels,0)
		else:
			labels=np.append(labels,1)
			sidenormal=np.subtract(p,np.mean(np.asarray(neighboors),axis=0))
			sidenormal=sidenormal/np.linalg.norm(sidenormal)
			sidenormals.append(sidenormal.tolist())
	labels=labels[1:]
	side_normals=np.asarray(sidenormals)
	max_label = 1
	c=[[1, 0, 0],[0,1,0]] #0=red 1=green
	pcds=[]
	for v in range(max_label+1):
		out_pcd = o3d.geometry.PointCloud()
		#print(labels)
		out_pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[labels == v])
		#out_pc.colors = o3d.utility.Vector3dVector(c[v])
		out_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[labels == v])
		#out_pc.paint_uniform_color(c[v])
		out_pcd.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[labels == v])
		pcds.append(out_pcd)
	print("\n\t{} outside points and {} inside points".format(len(pcds[1].points),len(pcds[0].points)))
	return pcds,side_normals

def extrusion(pcd,pcds,sidenormals,height):
	bottom=copy.deepcopy(pcd)
	points=np.asarray(bottom.points)
	vertex=np.asarray(bottom.normals)
	colors=np.asarray(bottom.colors)

	labels=np.ndarray([])

	z=np.mean(points[:,2],axis=0)

	for i in range(len(points)): 
		points[i,2]=z-height + height/2
		vertex[i]=-vertex[i]

	knew = o3d.geometry.PointCloud()
	knew.points = o3d.utility.Vector3dVector(points)
	knew.paint_uniform_color([1,0,1])
	knew.normals = o3d.utility.Vector3dVector(-vertex)
	knew.colors=o3d.utility.Vector3dVector(colors)
	pcds.append(knew)
	
	g=copy.deepcopy(pcds[1])
	points=np.asarray(g.points)
	vertex=np.asarray(sidenormals) #=>rather compute them for side !
	colors=np.asarray(g.colors)
	side_point=points
	side_color=colors
	side_vertex=vertex
	print(len(side_point))
	print(len(side_vertex))
	for v in range(20):
		for i in range(len(points)): 
			points[i,2]=(z-height)*(v+1)/21 + height/2
			#vertex[i,2]=0
		h = o3d.geometry.PointCloud()
		h.points = o3d.utility.Vector3dVector(points)
		h.normals = o3d.utility.Vector3dVector(sidenormals)
		h.colors = o3d.utility.Vector3dVector(colors)
		pcds.append(h)
		side_point=np.concatenate((side_point, h.points), axis=0)
		side_color=np.concatenate((side_color, h.colors), axis=0)
		side_vertex=np.concatenate((side_vertex, h.normals), axis=0)
		print(len(side_point))
		print(len(side_vertex))
	
	sides = o3d.geometry.PointCloud()
	sides.points = o3d.utility.Vector3dVector(side_point)
	sides.colors = o3d.utility.Vector3dVector(side_color)
	sides.normals = o3d.utility.Vector3dVector(side_vertex)	
	return bottom, sides

def save_total_pointcloud(num,top,bottom,edges,showing=False):
	#o3d.visualization.draw_geometries([top,bottom,edges])
	final=top+bottom+edges
	##final.estimate_normals()
	#o3d.geometry.estimate_normals(final)
	"""
	points=np.concatenate((top.points, edges.points, bottom.points), axis=0)
	colors=np.concatenate((top.colors, edges.colors, bottom.colors), axis=0)
	vertex=np.concatenate((top.normals, edges.normals, bottom.normals), axis=0)
	o3d.visualization.draw_geometries([top,bottom,edges])
	final = o3d.geometry.PointCloud()
	final.points = o3d.utility.Vector3dVector(points)
	final.colors = o3d.utility.Vector3dVector(colors)
	final.normals = o3d.utility.Vector3dVector(vertex)
	"""
	if showing:
		o3d.visualization.draw_geometries([final])
	o3d.io.write_point_cloud("data/treatment/step2_ShapeCompletion/"+str(num)+".ply", final, write_ascii=False, compressed=False, print_progress=True)
	return

def shape_completion(num_of_objects,threshold):
	print("Shape completion...")
	poses=[]
	for num in range(num_of_objects):
		#get pointcloud
		model="data/treatment/step1_Segmentation/"+str(num)+".ply"
		pcd = o3d.io.read_point_cloud(model)
		pcd=pcd.voxel_down_sample(voxel_size=0.003)
			
		print(6*"\n")
		print(len(np.asarray(pcd.points).tolist()))
		print(len(np.asarray(pcd.normals).tolist()))
		print(6*"\n")
		
		#get object pose
		pose,height=get_object_pose(pcd,threshold)
		poses.append(pose)
		
		#get object edges
		pcds,sidenormals=edge_clustering(pcd,poses[num])
		
		#extrude object
		bottom,edges=extrusion(pcd,pcds,sidenormals,height)
		
		#save final pointcloud
		save_total_pointcloud(num,top=pcd,bottom=bottom,edges=edges,showing=False)
	return poses

	
############################################################################
######################## MESHING (.ply to .stl)
############################################################################
def meshing_with_o3d(num):
	model="data/treatment/step2_ShapeCompletion/"+str(num)+".ply"
	pcd = o3d.io.read_point_cloud(model)
	pcd.estimate_normals()

	#https://github.com/isl-org/Open3D/issues/1829

	alpha=0.03

	tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
	#o3d.visualization.draw_geometries([pcd,tetra_mesh], mesh_show_back_face=True)

	mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha, tetra_mesh, pt_map)
	mesh.compute_vertex_normals()
	#o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

	#http://www.open3d.org/docs/0.12.0/python_api/open3d.io.write_triangle_mesh.html
	o3d.io.write_triangle_mesh("data/treatment/step3_Meshing/"+str(num)+".stl", mesh, write_ascii=False, compressed=False,write_vertex_normals=True, write_vertex_colors=True, print_progress=True)
	return

def meshing_with_pymeshlab(num):
	#ref: https://gist.github.com/shubhamwagh/0dc3b8173f662d39d4bf6f53d0f4d66b
	ms = pymeshlab.MeshSet()
	ms.load_new_mesh("data/treatment/step2_ShapeCompletion/"+str(num)+".ply")
	#ms.compute_normals_for_point_sets()
	ms.surface_reconstruction_screened_poisson()
	ms.compute_color_transfer_vertex_to_face()
	
	#ms.compute_color_by_function_per_face()
	
	ms.compute_texcoord_by_function_per_vertex()
	ms.compute_texcoord_transfer_vertex_to_wedge()
	ms.compute_texcoord_parametrization_triangle_trivial_per_wedge()
	
	ms.compute_color_by_function_per_face()#define material in .mtl file (ref: https://people.sc.fsu.edu/~jburkardt/data/mtl/mtl.html)
	
	ms.compute_texmap_from_color(textname=f""+str(num))#define color map in .png file
		
	#ms.compute_color_transfer_face_to_vertex()
	#ms.compute_color_transfer_mesh_to_face()
	#ms.meshing_invert_face_orientation()
	ms.save_current_mesh("data/treatment/step3_Meshing/"+str(num)+".obj")
	return

def meshing(num_of_objects):
	print("Meshing...")
	
	for num in range(num_of_objects):
		#meshing_with_o3d(num) #=>previous method but backface problem
		meshing_with_pymeshlab(num)
	
	print("\tDone")
	return


############################################################################
######################## PyBullet Simulation
############################################################################

def scene_pybullet_simulation(poses,threshold,realtime=0):
	#realtime=0 for no physics simulation, else realtime=1
	
	
	
	### SIMULATION PARAMETERS:
	cid= p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	p.setPhysicsEngineParameter(numSolverIterations=10)
	p.setTimeStep(1. / 120.)
	logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")
	#useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)
	p.loadURDF("plane_transparent.urdf", useMaximalCoordinates=True)#"plane100.urdf"
	
	#enable rendering during creation.
	p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
	p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
	
	#disable tinyrenderer, software (CPU) renderer, we don't use it here
	p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
	
	#enable rgb-depth-segmentation images
	p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)
	p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 1)
	p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
	
	
	### CAMERA PARAMETERS:
	width = 128
	height = 128

	fov = 60
	aspect = width / height
	near = 0.02
	far = 1
	
	view_matrix = p.computeViewMatrix(
	    cameraEyePosition=[0,0,threshold[3]],
	    cameraTargetPosition=[0, 0, 0],
	    cameraUpVector=[0, 1, 0])	
	projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	
	p.resetDebugVisualizerCamera(cameraDistance=threshold[3], cameraYaw=0, cameraPitch=-89, cameraTargetPosition=[0, 0, 0])
	
	
	### OBJECTS SPAWNING:
	path="data/treatment/step3_Meshing/"
	camera_pose=np.asarray([0.0,0.0,threshold[3]])
	models=[]
	for num in range(len(poses)):
		pos=camera_pose+np.asarray(poses[num][0].tolist())
		models.extend([[path+str(num)+".obj",pos.tolist()]])
	
	shift = [0, -0.02, 0]
	meshScale = [1, 1, 1]
	objectIDs=[]
	for model in models:
		if len(model)>2:
			color=model[2]
		else:
			color=[0, 1, 0, 1]#green
			
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
		a=p.createMultiBody(baseMass=0.1,
		              baseInertialFramePosition=[0, 0, 0],
		              baseCollisionShapeIndex=collisionShapeId,
		              baseVisualShapeIndex=visualShapeId,
		              basePosition=model[1],
		              useMaximalCoordinates=True)
		objectIDs.append(a)

	for elem in objectIDs:
	  print("object ID {} is at pose :".format(elem)+ str(p.getBasePositionAndOrientation(elem)))
	
	
	
	# START SIMULATION
	p.stopStateLogging(logId)
	p.setGravity(0, 0, -10)
	p.setRealTimeSimulation(realtime)
	time.sleep(1)
	
	
	
	# SAVE IMAGES
	images = p.getCameraImage(width,
                          height,
                          view_matrix,
                          projection_matrix,
                          shadow=True,
                          renderer=p.ER_BULLET_HARDWARE_OPENGL)
	# NOTE: the ordering of height and width change based on the conversion
	rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
	depth_buffer_opengl = np.reshape(images[3], [width, height])
	depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
	seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.
	time.sleep(1)
	
	# Plot both images - should show depth values of 0.45 over the cube and 0.5 over the plane
	plt.rcParams['figure.figsize'] = [4, 10]	
	
	plt.subplot(6, 1, 1)
	plt.imshow(depth_opengl, cmap='gray', vmin=0, vmax=1)
	plt.title('Depth OpenGL3')

	plt.subplot(5, 1, 3)
	plt.imshow(rgb_opengl)
	plt.title('RGB OpenGL3')

	plt.subplot(5, 1, 5)
	plt.imshow(seg_opengl)
	plt.title('Seg OpenGL3')

	plt.imsave("data/treatment/step4_Simulation/depth.png", depth_opengl)
	plt.imsave("data/treatment/step4_Simulation/seg.png", seg_opengl)
	plt.imsave("data/treatment/step4_Simulation/rgb.png", rgb_opengl, cmap='gray', vmin=0, vmax=1)
	
	
	#DISPLAY POSITIONS & IMAGES
	while (1):
		for elem in objectIDs:
			print("object ID {} is at pose :".format(elem)+ str(p.getBasePositionAndOrientation(elem)))
		#plt.show()
		time.sleep(1)
	return
	


############################################################################
############################################################################
######################## TOTAL DETECTION & SIMULATION
############################################################################
############################################################################

def modelfree_detection_and_simulation():
	#1. data aquisition from realsense camera
	data_aquisition()
	
	#2. pointcloud segmentation
	num_of_objects,threshold=o3d_segmentation()
	
	#3. shape completion by extrusion to the tabletop plane
	poses=shape_completion(num_of_objects,threshold)
	
	#4. meshing
	meshing(num_of_objects)
	
	#5 pybullet simulation
	scene_pybullet_simulation(poses,threshold,realtime=1)
	
	return

modelfree_detection_and_simulation()



