steps for use on **host computer** with Tensorflow 2.5.0:

1. download mask_rcnn_coco.h5 via link: https://github.com/matterport/Mask_RCNN/releases/download/v2.0/mask_rcnn_coco.h5
2. optional : launch [verify_allisgood_before_training.py](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/verify_allisgood_before_training.py)
3. launch [train.py](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/train.py)
4. launch [show_training_results.py](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/show_training_results.py)

steps for use on **Colaboratory**:

1. download on your drive : [mrcnn.zip](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/mrcnn.zip) and [images.zip](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/images.zip)
2. optional : for better performances, select "Hardware accelerator: GPU" so as to access cuda and nvidia
3. launch [training_colaboratory.ipynb](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/training_colaboratory.ipynb) on colaboratory

