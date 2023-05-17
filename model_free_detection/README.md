The main program should be launched through the [main.py](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/model_free_detection/main.py) file.
mrcnn folder is requiered to apply the supervised segmentation method and robot_model for the introduction of the panda robot in the PyBullet simulation.

## Pre-steps
Download the neural network model, stored in [Google drive](https://drive.google.com/file/d/1n57O2nKsVMDgJbNW5ZxttaGIwd9qLF93/view?usp=sharing) due to its too size :
```bash
cd model_free_detection
wget https://drive.google.com/file/d/1n57O2nKsVMDgJbNW5ZxttaGIwd9qLF93/view?usp=sharing
```

## Deployment

To record a livestream:
```bash
cd model_free_detection
python3 main.py
```

To open a recording file *"filename.bag"*, put it in the folder *model_free_detection/data/recordings* and then run the following commands:
```bash
cd model_free_detection
python3 main.py filename.bag
```
