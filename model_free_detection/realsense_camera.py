import pyrealsense2 as rs
import numpy as np
import imageio #https://github.com/IntelRealSense/librealsense/issues/3658
import cv2


class RealsenseCamera:
    def __init__(self,recording=None,path_target="data/data_aquisition/"):
        print("Camera set up...")
        # Configure depth and color streams
        print("\tLoading Intel Realsense Camera D435f")
        
        # Assign path
        self.path_target=path_target
        
        # Create pipeline
        self.pipeline = rs.pipeline()
        # Create a config object
        config = rs.config()
        # Tell config that we will use a recorded device from file to be used by the pipeline through playback.
        if recording!=None:
            self.recording=True
            rs.config.enable_device_from_file(config,str(recording))
            print("\tuse recording :'"+str(recording)+"'")
        else:
            self.recording=False
        # Configure the pipeline to stream the depth stream
        # Change this parameters according to the recorded bag file resolution
        
        if self.recording:
            #https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/read_bag_example.py
            #https://github.com/IntelRealSense/librealsense/issues/4167
            pipe_profile = self.pipeline.start(config)
            playback = pipe_profile.get_device().as_playback()
            #config.enable_stream(rs.stream.depth, rs.format.z16, 30)
            #config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)#848, 480 for D355
            config.enable_stream(rs.stream.color,  1280, 720, rs.format.bgr8, 30)#1280, 720 for D455
            # Start streaming from file
            self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        # Create opencv window to render image in
        #cv2.namedWindow("Depth Stream", cv2.WINDOW_AUTOSIZE)    
        # Create colorizer object
        self.colorizer = rs.colorizer()
        self.colorized=None
        self.frames=None
        print("\tDone")


    def get_frame_stream(self):
        print("Get frame stream...")
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        self.frames=frames
        
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        
        if self.recording:
            color_frame = aligned_frames.get_color_frame()
            #color_frame = self.colorizer.colorize(depth_frame)
        else:
            color_frame = aligned_frames.get_color_frame()
        
        self.frames=aligned_frames
        
        if not self.recording and (not depth_frame or not color_frame):
            # If there is no frame, then camera not connected, return False
            print("\tError, impossible to get the frame, make sure that the Intel Realsense camera is correctly connected")
            return False, None, None
        
        # Apply Spatial filter to fill the Holes in the depth image
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(depth_frame)
        
        # Apply Temporal filter to fill the Holes in the depth image
        temporal = rs.temporal_filter()
        temp_filtered_depth = temporal.process(filtered_depth)
        
        #Apply Hole filling
        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(temp_filtered_depth)
        
        # Create colormap to show the depth of the Objects
        #colorizer = rs.colorizer()
        depth_colormap = np.asanyarray(self.colorizer.colorize(filled_depth).get_data())
        
        self.colorized = self.colorizer.process(frames)
        
        # Convert images to numpy arrays
        distance = depth_frame.get_distance(int(50),int(50))
        # print("distance", distance)
        depth_image = np.asanyarray(filled_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        """print(depth_image)
        print(np.max(depth_image))
        print(np.min(depth_image))
        print(np.unique(depth_image))"""
        f=open(self.path_target+"depth/4.txt",'w+')
        for w in range(len(depth_image)):
        	for z in range(len(depth_image[0])):
        		f.write(str(depth_image[w][z])+" ")
        	f.write("\n")
        f.close()
        
        """k=np.unique(depth_image)
        
        import copy
        kk=copy.deepcopy(k).tolist()
        a=[0,0,0]
        j=0
        for i in range(len(todo)):
        	kk[i]=copy.deepcopy(a)
        	j+=1
        	if j>2:
        		j=0
        	a[j]+=1
        for i in range(len(depth_image)):
        	for j in range(len(depth_image[0])):
        		depth_image[i][j]=depth_image[i][j]
        
        #print(todo)
        print(todo)"""
        todo=cv2.convertScaleAbs(depth_image)
        """print(todo)
        print(depth_image)
        print(todo.dtype)
        print(depth_image.dtype)"""
        a=np.where(depth_image < 2000, depth_image,255)
        for w in range(len(depth_image)):
        	for z in range(len(depth_image[0])):
        		todo[w][z]=a[w][z]
        """print(np.unique(a))
        print(np.unique(todo))"""
        depth_image = cv2.applyColorMap(todo, cv2.COLORMAP_JET)
        
        #print(depth_image)
        '''
        z=cv2.convertScaleAbs(depth_image, alpha=0.3)
        print(z)
        print(np.max(z))
        print(np.min(z))
        print(np.unique(z))'''
        #depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_HSV)#COLORMAP_JET#https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html#ga9a805d8262bcbe273f16be9ea2055a65
        
        '''print('haaaa')
        k=np.unique(depth_image)
        
        #depth_image = np.where(depth_image>=k[-1], 0, an_array)
        
        depth_image=depth_image-(k[1]-1)*np.ones(depth_image.shape)
        depth_image=np.where(depth_image >= 0, depth_image, 0)
        #k=np.unique(depth_image)
        #depth_image=np.where(depth_image < k[254], depth_image, k[254])
        print(np.unique(depth_image))
        depth_image=depth_image/np.max(depth_image)
        print(np.unique(depth_image))
        depth_image=255*depth_image
        print(np.unique(depth_image))
        depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image), cv2.COLORMAP_JET)
        '''
        
        
        print("\tframe OK\n\tDone")
        
        return True, color_image, depth_image
        
        
    #https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/export_ply_example.py
    def saveply(self,name,color_frame,depth_frame):
        print("\tSave ply...")
        ply = rs.save_to_ply(self.path_target+"ply/"+name+".ply")
        ply.set_option(rs.save_to_ply.option_ply_binary, False)
        ply.set_option(rs.save_to_ply.option_ply_normals, True)
        #ply.set_option(rs.save_to_ply.option_ignore_color, False)
        #ply.set_option(rs.save_to_ply.option_ply_mesh, False)
        ply.process(self.frames)#ply.process(self.colorized) for depth color map
        return
    
    #https://github.com/IntelRealSense/librealsense/issues/3658
    def saveframe(self, name, frame):
        print("\tSave frame '"+name+"'...")
        image = np.asanyarray(frame)
        imageio.imwrite(self.path_target+name+".png", image)
        return
    
    
    
    def release(self):
        self.pipeline.stop()
