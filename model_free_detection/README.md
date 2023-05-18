The main program should be launched through the [main.py](https://github.com/LouiseMassager/PandaPush_Depth_Reconstruction/blob/master/model_free_detection/main.py) file.
mrcnn folder is requiered to apply the supervised segmentation method and robot_model for the introduction of the panda robot in the PyBullet simulation.

## Pre-steps
Download the **neural network model**, stored in [Google drive](https://drive.google.com/file/d/1HYM2qZfUeh4nNsfIYNdz92wCEfzZJ6Xp/view?usp=sharing) due to its size (**250Mo** > GitHub's limit of 100 Mo) :
```bash
cd model_free_detection
wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1HYM2qZfUeh4nNsfIYNdz92wCEfzZJ6Xp' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1HYM2qZfUeh4nNsfIYNdz92wCEfzZJ6Xp" -O mask_rcnn_cubecyl2.h5 && rm -rf /tmp/cookies.txt
```

## Deployment

To record a **livestream**:
```bash
cd model_free_detection
python3 main.py
```

To **run the program on a recording** stored in a file *"filename.bag"*, put it in the folder *model_free_detection/data/recordings* and then run the following commands:
```bash
cd model_free_detection
python3 main.py filename.bag
```
