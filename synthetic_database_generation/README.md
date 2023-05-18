## Pre-design steps

In order to adapt the synthetic database, this program should be editted depending on the requierements. One may want to:
- **changing the size of the dataset** : The [*make_scene.py*](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/synthetic_database_generation/make_scene.py) script can be run as it is to do so. The user will simply need to enter the number of images to produce in the terminal when asked to.<br />

- **changing the complexity/diversity of the scenes** : Different features are already provided in the program [*make_scene.py*](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/synthetic_database_generation/make_scene.py) (eg. place objects randomly or close from one to another, realise scene with the same object with different colors or with different objects, etc.). The type of objects (in our case cuboids or cylinders), their maximum number in a scene and other parameters are also chosen by the user during the excecution of this script. The script can thus be launched as it is or new features can be added to create new types of scenes.<br />

- **changing the complexity/diversity of the objects to detect**: similarly to above, the script [*make_shapes.py*](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/synthetic_database_generation/make_shapes.py) also contain parameters to tune (eg. random, realistic or no noise/ dimensions of cubes and cylinders / shadows in colors).<br />

- **changing/adding objects to detect**: lines 582 to 596 of [*make_scene.py*](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/synthetic_database_generation/make_scene.py) should be adapted. The new objects to consider can then either be already provided in CAD or generated/randomly modified via modifications in the [*make_shapes.py*](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/synthetic_database_generation/make_shapes.py) script. Small changes in the training code will also be necessary : please refer to the README file in [*"ML_training"*](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/tree/master/ML_training) folder. 

## Deployement

1. Generate random objects CAD:
```bash
cd synthetic_database_generation
python3 make_shapes.py
```

2. Generate the database through random scenes generated in a PyBullet simulation:
```bash
cd synthetic_database_generation
python3 make_scene.py
```
## Results

This program should resutl in a new dataset stored in the *"image"* folder. The following steps to include the changes in the main program would be to retrain the neural network with this new dataset: see [*"ML_training"*](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/tree/master/ML_training) folder.
