import mrcnn
import mrcnn.config
import mrcnn.model
import mrcnn.visualize
import cv2
import os
import numpy as np

CLASS_NAMES = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']

class SimpleConfig(mrcnn.config.Config):
    NAME = "coco_inference"
    
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

    NUM_CLASSES = len(CLASS_NAMES)

model = mrcnn.model.MaskRCNN(mode="inference", 
                             config=SimpleConfig(),
                             model_dir=os.getcwd())

model.load_weights(filepath="mask_rcnn_coco.h5", 
                   by_name=True)

image = cv2.imread("sample2.jpg")
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

r = model.detect([image], verbose=0)

r = r[0]


cv2.imwrite('sample2_image.jpg',image)
mask=r['masks']
print(mask)
print(mask.shape)
print(mask.dtype)
"""
#cv2.imwrite('sample2_mask.jpg',mask)

i=0
for z in range(len(mask[0][0])):
    	mask_i= mask[:, : ,[z]]
    	mask_i=np.squeeze(mask_i)
    	#print(mask_i.shape)
    	#print(mask_i.dtype)
    	mask_i = mask_i.astype(np.int32)
    	for a in range(len(mask_i)):
    		for b in range(len(mask_i[0])):
    			#print(mask_i[a][b])
    			if mask_i[a][b]:
    				mask_i[a][b]=0
    				print("HERE!")
    				i+=1
    				#print(mask_i[a][b])
    			else:
    				mask_i[a][b]=255
    	print(mask_i)
    	#print(mask_i.dtype)
    	#print(mask_i.shape)
    	print(i)
    	cv2.imwrite('res/image_mask'+str(z)+'.jpg',mask_i)
    	



"""
mrcnn.visualize.display_instances(image=image, 
                                  boxes=r['rois'], 
                                  masks=r['masks'], 
                                  class_ids=r['class_ids'], 
                                  class_names=CLASS_NAMES, 
                                  scores=r['scores'])



