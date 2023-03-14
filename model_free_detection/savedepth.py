import cv2

import open3d as o3d

import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
import copy

import cv2
import pymeshlab

name="total_bag3"
model=name+".ply"
pcd = o3d.io.read_point_cloud(model)

c=np.mean(np.asarray([pcd.points]),axis=1)
print(c)


pcdfloor=copy.deepcopy(pcd).translate((0, 0, -0.01), relative=True)
pcd=pcd+pcdfloor
#o3d.visualization.draw_geometries([pcd])
model="a.ply"
o3d.io.write_point_cloud(model, pcd, write_ascii=False, compressed=False, print_progress=True)


ms = pymeshlab.MeshSet()
ms.load_new_mesh(model)
#ms.surface_reconstruction_screened_poisson()
ms.surface_reconstruction_ball_pivoting()
ms.save_current_mesh("test.ply")
model="test.ply"


img_width, img_height = (1280, 720)
pcd = o3d.io.read_triangle_mesh(model)


mat = o3d.visualization.rendering.MaterialRecord()

mat.shader = 'defaultUnlit'

renderer_pc = o3d.visualization.rendering.OffscreenRenderer(img_width, img_height)
renderer_pc.scene.set_background(np.array([0, 0, 0, 0]))
renderer_pc.scene.add_geometry("pcd", pcd, mat)

# Optionally set the camera field of view (to zoom in a bit)
vertical_field_of_view = 15.0  # between 5 and 90 degrees
aspect_ratio = img_width / img_height  # azimuth over elevation
near_plane = 0.1
far_plane = 50.0
fov_type = o3d.visualization.rendering.Camera.FovType.Vertical
renderer_pc.scene.camera.set_projection(vertical_field_of_view, aspect_ratio, near_plane, far_plane, fov_type)

# Look at the origin from the front (along the -Z direction, into the screen), with Y as Up.
center =c[0]# [0, 0, 0]  # look_at target
eye = [0,0,0.5]#[0, -0.2, 1]  # camera position
up = [0, 1, 0]  # camera orientation
#center= [0.0149067  ,0.1074661,  -0.43860744]
renderer_pc.scene.camera.look_at(center, eye, up)

depth_image = np.asarray(renderer_pc.render_to_depth_image())
print(depth_image)
print(np.unique(depth_image))
np.save('depth', depth_image)

normalized_image = depth_image#(depth_image - depth_image.min()) / (depth_image.max() - depth_image.min())
plt.imshow(normalized_image)
#plt.savefig(name+'_depth.png')
plt.imsave(name+'_depth.jpg', depth_image)





