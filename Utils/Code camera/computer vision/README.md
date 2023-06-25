# Ball Tossing Arm - Computer Vision Repository

This repository contains all the code developed during the semester in the context of the Project from P19 - Robotics, Ball Tossing Arm.

# Setup

Before using the scripts, it's recommended to setup a Python virtual environment.

To install the required libraries, run
  ```console
    $ pip install -r requirements.txt
  ```

# Scripts

The following section gives a brief description of every script present in the repository.

- `aruco_generator.py`: Generates an ArUco marker image of the desired id and type.
- `camera_calibration.py`: Generates the camera calibration information (intrinsic matrix, distortion coefficients, tvecs and rvecs). Two versions were created to solve incomatibility issues.
- `functions.py`: Contains all the functions used in the process of distance and angle estimation.
- `picture_taker.py`: Captures an image and saves it locally.
- `image_detection.py`: Estimates the cup distance and angle from the robotic arm using an input image. Assumes both the arm and the cup have ArUco markers attached.
- `distance_markers.py`: Draws distance between the arm marker and the cup marker in real-time.
- `video_detection.py`: Draws the bounding box detections of the YOLO algorithm in real-time.
- `video_detection_yolov5.py`: Attempts to do the same process mentioned before, but using a YOLOv5 custom model, trained to only detect cups. Not currently working.
- `visualize_3d.py`: Contains a function to visualize 3D points interactively. Helpful to better understand perspective transformations using ArUco markers.
- `ros_com`: This directory contains the publisher and subscriber meant to be used in the transmition of information from and to the simulation. The subscriber isn't finished, and the publisher is using placeholder values.