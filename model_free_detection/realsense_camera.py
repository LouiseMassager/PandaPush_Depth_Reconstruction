import pyrealsense2 as rs
import numpy as np
import imageio #https://github.com/IntelRealSense/librealsense/issues/3658
import cv2


class RealsenseCamera:
    def __init__(self,recording=None):
        print("Camera set up...")
        # Configure depth and color streams
        print("\tLoading Intel Realsense Camera D435f")
        
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
        print("\tframe OK\n\tDone")
        return True, color_image, depth_image
        
        
    #https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/export_ply_example.py
    def saveply(self,name,color_frame,depth_frame):
        print("\tSave ply...")
        ply = rs.save_to_ply("data/ply/"+name+".ply")
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
        imageio.imwrite("data/"+name+".png", image)
        return
    
    
    
    def release(self):
        self.pipeline.stop()
