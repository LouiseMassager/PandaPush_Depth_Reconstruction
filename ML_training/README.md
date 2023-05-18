## Pre-design steps
A new training can be realized to:
- **improve performances** of the segmentation by realising a **more intensive training**. Parameters in the [training script](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/training_colaboratory.ipynb) should be then tuned  (more epoch,lower learning rates, etc.). They are mainly defined upon call of the model.train() function in the *"TRAINING"* section. Following the changes in parameters, the code can be run as it is (see Deployement steps).
- **improve performances** of the segmentation by **changing the dataset** (to have more images, different images, etc.). A new *"images.zip"* folder should then be used and the code run here as it is (see Deployement steps). The new dataset can be produced through the launch of the [synthetic database generator scripts](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/tree/master/synthetic_database_generation).
- **adding/changing the objects to detect**. In order to do so, the synthetic database should be adapted to contain different labels in the segmentation text in *images/seg* folder (currently each line starting with "0" or a "1" correspond respectively to a cuboid or a cylinder). The number of classes can be decreased/increased but its numbering should start at 0 and increment of 1 for each new class. This change can be realised in lines 591 to 596 of the [*make_scene.py*](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/synthetic_database_generation/make_scene.py) script. The following changes in before its launch:
> in *"CONFIGURATIONS-Training Configurations"* section: NUM_CLASSES should be equal to the number of classes + 1 
>> &rarr NUM_CLASSES = 1 + 2  # background + cube&cylinder

> in *"CONFIGURATIONS-Dataset Adaptation"* section:  the number of lines and significations should change to match the new classes
>> &rarr self.add_class("shapes", 1, "cube")
>> &rarr self.add_class("shapes", 2, "cylinder")

## Deployement
steps for use on **host computer** with Tensorflow 2.5.0:

1. download mask_rcnn_coco.h5 via link: https://github.com/matterport/Mask_RCNN/releases/download/v2.0/mask_rcnn_coco.h5
2. optional : launch [verify_allisgood_before_training.py](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/verify_allisgood_before_training.py)
3. launch [train.py](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/train.py)
4. launch [show_training_results.py](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/show_training_results.py)



steps for use on **Colaboratory**:

1. download on your drive : [mrcnn.zip](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/mrcnn.zip) and [images.zip](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/images.zip)
2. optional : for better performances, select "Hardware accelerator: GPU" so as to access cuda and nvidia
3. launch [training_colaboratory.ipynb](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/ML_training/training_colaboratory.ipynb) on colaboratory

## Results

This training should result in the creation of a new model stored in a .h5 files (default name: *"mask_rcnn_cubecyl2.h5"*). In order to incorporate this new model into the chore program, one just need to replace the current model (.h5) file in *"model_free_detection/"* by the new model.
