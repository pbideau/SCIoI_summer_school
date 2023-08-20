import cv2
from picamera2 import Picamera2
import torch
import cv2
import os 
import glob
import argparse
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from libcamera import Transform

#YOLO_MODEL = '../model/yolo_model.pt'
YOLO_MODEL = '../model/nano_model.pt'
FILE = './data/take0_17-05-34/2023-6-28_17-05-34-821/2023-6-28_17-06-35-343099.png'
DEVICE = "cpu"
ROBOT_HEIGHT_M = 0.12
#FOCALLENGTH_PX = 528.695652 # picamera v2
FOCALLENGTH_PX = 627.590698 #470.325581 # picamera v3
CONFIDENCE = 0.5


def dist(bounding_boxes, focallength_px, robot_height_m):
	
	distance = []
	
	for box in bounding_boxes:
		
		F1 = math.sqrt(FOCALLENGTH_PX**2 + (box[3]-240)**2 + (box[0]-320)**2)
		F2 = math.sqrt(FOCALLENGTH_PX**2 + (box[1]-240)**2 + (box[1]-320)**2)
		b = box[3]/F1 - box[1]/F2
		dist = ROBOT_HEIGHT_M / b
		print(dist)
		
		distance.append(dist)
			
	return distance
	
	
def visualize(im, bounding_boxes, distance):
	
	i = 0
	
	for box in bounding_boxes:

		img_bb = cv2.rectangle(im, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
		image_bb_label = cv2.rectangle(img_bb, (int(box[0]), int(box[1])-20), (int(box[0]) + 70, int(box[1])), (255,255,255), -1)
		image_bb_label_text = cv2.putText(image_bb_label, f"Dist: {(distance[i]):.3f}", (int(box[0]), int(box[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0), 1, cv2.LINE_AA)
		image_result = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
		i = i+1
	
	return image_result
	
		
    
if __name__ == '__main__':

	# Loading YOLOv5 Model
	model_YOLO = torch.hub.load('ultralytics/yolov5', 'custom', path=YOLO_MODEL, verbose=False)
	model_YOLO.conf = CONFIDENCE
	model_YOLO.to(DEVICE)

	cv2.startWindowThread()

	# start and configure picamera
	picam2 = Picamera2()
	#picam2.configure(picam2.create_preview_configuration(main={"format": 'BGR888', "size": (640, 480)})) #picamera v2
	picam2.configure(picam2.create_preview_configuration(main={"format": 'BGR888', "size": (1536, 864)})) #picamera v3
	picam2.start()
	
	# info to resize perceived image to the same aspect ratio as it was trained on
	width = math.ceil(480 * 1536/864) #360
	height = 480
	dim = (width, height)
	dx = int((width - 640)/2)

	while True:
		
		im = picam2.capture_array()
		im = cv2.rotate(im, cv2.ROTATE_180)	
		
		# resize frame
		im = cv2.resize(im, dim, interpolation = cv2.INTER_AREA)
		im = im[:, dx:width-dx,:]
		
		# run detection on image frame
		res = model_YOLO(im) 
		bounding_boxes = res.xyxy[0][res.xyxy[0][:, 0].sort()[1]]
		
		if bounding_boxes.size(dim=0) == 0:
			
			print("no robots detected.")
			
			# visualize result: plain image
			image_result = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
			
		else:

			# compute distance
			distance = dist(bounding_boxes, FOCALLENGTH_PX, ROBOT_HEIGHT_M)
			
			# visualize result: image with detection boxes and distance estimates
			image_result = visualize(im, bounding_boxes, distance)

		cv2.imshow("Camera", image_result)
		cv2.waitKey(1)
