# PandaPush_RL_CV :video_camera: :game_die: :mechanical_arm: :robot:
```diff
- project ongoing
```
Project on the detection of 3D objects with a ZED2 Depth camera based on a classifier (Machine Learning) trained with a 3D model of the object.
Potentially tested for the control of a Panda robot for non-prehensile manipulation (push tasks) with those 3D objects.

# Updates
2022/11/20 - mesh convertisser from 3D model (.stl) to point cloud (.ply) and vis versa

## Resources
#### For this project
- [Colaboratory](https://colab.research.google.com/?utm_source=scs-index) : for working environment (provide online GPU)
- [Mediapipe](https://google.github.io/mediapipe/solutions/objectron) OR [Deep_Object_Pose](https://github.com/NVlabs/Deep_Object_Pose) : for detection of object based on 3D model (with ML training) <br />
[Mediapipecodeexample](https://www.youtube.com/watch?v=f-Ibri14KMY&ab_channel=NicolaiNielsen-ComputerVision%26AI)

#### For general knowledge
- [Yolo](https://pjreddie.com/darknet/yolo/) : Real-Time Object Detection (for common knowledge)


<br /> <br /> <br />
## Packages
- python3

#### Computer Vision (CV)
- tensorflow
- opencv
- gym

#### Simulation
- pybullet
- ffmpeg : for recording
