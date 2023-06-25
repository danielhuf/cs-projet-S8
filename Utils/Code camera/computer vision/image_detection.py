""" 
How to run the snippet:
python image_detection.py --image img/saved_img.jpg --type DICT_7X7_50 
python image_detection.py --image img/simu.png --type DICT_7X7_50

Optional arguments:
--calibration: path to the calibration folder
"""

import cv2
import numpy as np
import argparse
from functions import getArucoTag, pose_estimation, get_cup_3d_position, undistort_image, \
    estimate_cup_depth, estimate_cup_3d_position, project2Dinto3D

print("[INFO] constructing argument parser...")
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="path to input image containing cup")
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to detect")
ap.add_argument("-c", "--calibration", type=str, default="calibration", help="path to the calibration folder")
args = vars(ap.parse_args())

print("[INFO] reading yolo classes...")
net = cv2.dnn.readNet("yolo/yolov3.weights", "yolo/yolov3.cfg")
classes = []
with open("yolo/coco.names", "r") as f:
    classes = f.read().splitlines()

print("[INFO] loading camera calibration data...")
intrinsic_camera = np.load(f'{args["calibration"]}/cameraMatrix.npy')
distortion = np.load(f'{args["calibration"]}/dist.npy')

print("[INFO] loading image...")
img = undistort_image(cv2.imread(args["image"]), intrinsic_camera, distortion)
img2 = cv2.imread(args["image"])
print("[INFO] Undistorted image is equal to original image: ", np.array_equal(img, img2))

height, width, _ = img.shape

blob = cv2.dnn.blobFromImage(img, 1/255, (416, 416), (0,0,0), swapRB=True, crop=False)
net.setInput(blob)

output_layers_names = net.getUnconnectedOutLayersNames()
layerOutputs = net.forward(output_layers_names)

boxes = []
confidences = []
class_ids = []

print("[INFO] Size of layerOutputs: ", len(layerOutputs))
for output in layerOutputs:
    print("[INFO] Size of output: ", len(output))
    for detection in output:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > 0.5 and classes[class_id] == "cup":

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

            break

indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

font = cv2.FONT_HERSHEY_PLAIN
colors = np.random.uniform(0, 255, size=(len(boxes), 3))
x_mid = None
y_lowmid = None
if len(indexes)>0:
    for i in indexes.flatten():

        # Placing box in the detected object
        x, y, w, h = boxes[i]
        label = str(classes[class_ids[i]])
        confidence = str(round(confidences[i],2))
        color = colors[i]
        cv2.rectangle(img, (x,y), (x+w, y+h), color, 2)
        cv2.putText(img, label + " " + confidence, (x, y+20), font, 2, (255,255,255), 2)

        # Positions of the upper middle point
        x_mid = int(center_x)
        y_upmid = int(center_y - h/2)
        y_mid = int(center_y)

        # Draw point in the lower middle of the cup
        y_lowmid = int(center_y + h/2)
        cv2.circle(img, (x_mid, y_lowmid), 5, (0,0,255), -1)

        # Add a label to the point
        text = "original"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (255, 255, 255) # BGR color format
        thickness = 3
        text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
        text_origin = (x_mid - text_size[0] // 2, y_lowmid - text_size[1] - 5)
        cv2.putText(img, text, text_origin, font, font_scale, color, thickness)

        # Get 3d position of the cup
        #cup_3d_position = get_cup_3d_position(x_mid, y_upmid, w)
        #print("*** Cup 3d position: ", cup_3d_position)
        cup_depth = estimate_cup_depth(w, h, 8.7, 8.7, intrinsic_camera)
        print("*** Cup depth: ", cup_depth)
       
        cup_3d_position = estimate_cup_3d_position(x_mid, y_lowmid, cup_depth, intrinsic_camera)


        print("*** Cup 3d position: ", cup_3d_position)


if x_mid is None or y_lowmid is None:
    print("[ERROR] No cup detected")
else:
    #cup = np.array([x_mid, y_lowmid], dtype=np.float32)
    cup_marker_id = 0
    arm_marker_id = 1
    output, cup_3d_pos, arm_3d_pos, arm_cup_distance, angle = pose_estimation(img, getArucoTag(args["type"]), intrinsic_camera, distortion, arm_marker_id, cup_marker_id, x_mid, y_mid, cup_depth)

    cv2.imshow('Scenario', cv2.resize(output, (960, 540)) )
    print("*** Cup 3d position: ", cup_3d_pos)
    print("*** Arm 3d position: ", arm_3d_pos)
    print("*** Arm cup distance: ", arm_cup_distance)
    print("*** Angle: ", angle)

    cv2.waitKey(0)
    cv2.destroyAllWindows()