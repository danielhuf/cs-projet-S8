""" 
How to run the snippet:
python video_detection.py --type DICT_7X7_50 
"""

import argparse
import cv2
import numpy as np
from functions import *
import time

def format_yolov5(source):

    # put the image in square big enough
    col, row, _ = source.shape
    _max = max(col, row)
    resized = np.zeros((_max, _max, 3), np.uint8)
    resized[0:col, 0:row] = source
    
    # resize to 640x640, normalize to [0,1[ and swap Red and Blue channels
    result = cv2.dnn.blobFromImage(resized, 1/255.0, (640, 640), swapRB=True)
    
    return result

frame_count = 0
fps_list = []

print("[INFO] constructing argument parser...")
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

print("[INFO] reading yolo classes...")
net = cv2.dnn.readNet("yolo/best.onnx")
classes = []
with open("yolo/coco2.names", "r") as f:
    classes = f.read().splitlines()

print("[INFO] loading camera calibration data...")
intrinsic_camera = np.load('calibration/cameraMatrix.npy')
distortion = np.load('calibration/dist.npy')

print("[INFO] turning on camera...")
cap = cv2.VideoCapture(0)

start_time = time.time()

while cap.isOpened():
	_, img = cap.read()
	height, width, _ = img.shape

	blob = format_yolov5(img) #cv2.dnn.blobFromImage(img, 1/255, (416, 416), (0,0,0), swapRB=True, crop=False)
	net.setInput(blob)

	output_layers_names = net.getUnconnectedOutLayersNames()
	layerOutputs = net.forward(output_layers_names)

	boxes = []
	confidences = []
	class_ids = []

	for output in layerOutputs:
		for detection in output:
			scores = detection[5:]
			print("scores:", scores)
			class_id = np.argmax(scores)
			confidence = scores[class_id]
			if confidence > 0.5:
				center_x = int(detection[0]*width)
				center_y = int(detection[1]*height)
				w = int(detection[2]*width)
				h = int(detection[3]*height)

				# Positions of the upper left corner
				x = int(center_x - w/2)
				y = int(center_y - h/2)

				boxes.append([x, y, w, h])
				confidences.append((float(confidence)))
				class_ids.append(class_id)

	indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

	font = cv2.FONT_HERSHEY_PLAIN
	colors = np.random.uniform(0, 255, size=(len(boxes), 3))

	if len(indexes)>0:
		for i in indexes.flatten():
			x, y, w, h = boxes[i]
			label = str(classes[class_ids[i]])
			confidence = str(round(confidences[i],2))
			color = colors[i]
			#if label == "cup":
			cv2.rectangle(img, (x,y), (x+w, y+h), color, 2)
			cv2.putText(img, label + " " + confidence, (x, y+20), font, 2, (255,255,255), 2)

	#output = pose_estimation(img, getArucoTag(args["type"]), intrinsic_camera, distortion)
	
	cv2.imshow('Scenario', img)
	frame_count += 1
	current_time = time.time()
	elapsed_time = current_time - start_time
	if elapsed_time > 1:
		fps = frame_count / elapsed_time
		print("FPS:", fps)
		# Reset the variables
		frame_count = 0
		start_time = current_time

		fps_list.append(fps)



	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		break

print("[INFO] Average FPS:", sum(fps_list)/len(fps_list))


cap.release()
cv2.destroyAllWindows()