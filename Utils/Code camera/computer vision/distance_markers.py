""" 
How to run the snippet:
python distance_markers.py --image img/saved_img.jpg --type DICT_7X7_50 
python distance_markers.py --image img/simu.png --type DICT_7X7_50

Optional arguments:
--calibration: path to the calibration folder
"""

import cv2
from cv2 import aruco
import numpy as np
import argparse
from functions import (getArucoTag, draw_height_cup_marker, get_marker_cup_angle,
                       project3Dinto2D, cam_to_marker_coords, 
                       get_cup_center, marker_to_cam_coords, pose_estimation) 

import time


print("[INFO] constructing argument parser...")
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="path to input image containing cup")
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to detect")
ap.add_argument("-c", "--calibration", type=str, default="calibration", help="path to the calibration folder")
args = vars(ap.parse_args())

print("[INFO] loading camera calibration data...")
cam_mat = np.load(f'{args["calibration"]}/cameraMatrix.npy')
dist_coef = np.load(f'{args["calibration"]}/dist.npy')
r_vectors = np.load(f'{args["calibration"]}/rvecs.npy')
t_vectors = np.load(f'{args["calibration"]}/tvecs.npy')

MARKER_SIZE = 13  # centimeters

arucoDict = cv2.aruco.getPredefinedDictionary(getArucoTag(args["type"])) # old Dictionary_get
arucoParams = cv2.aruco.DetectorParameters() # old DetectorParameters_create


cap = cv2.VideoCapture(0)

frame_count = 0
fps_list = []
start_time = time.time()

def projectMarker(marker_3d, cam_mat, dist_coef):
    '''
    Project a 3D Aruco marker onto the image plane
    marker_3d = np.array([x, y, z])
    '''
    # Define the 3D position of the ArUco marker
    
    # Reshape the marker_3d array to be a 1x1x3 matrix
    marker_3d = np.reshape(marker_3d, (1, 1, 3))

    # Perform projection from 3D to 2D
    marker_2d, _ = cv2.projectPoints(marker_3d, np.zeros((3, 1)), np.zeros((3, 1)), cam_mat, dist_coef)

    # Extract the 2D position of the marker
    marker_2d = marker_2d.squeeze()

    # Convert the marker_2d coordinates to integer values
    marker_2d = (int(marker_2d[0]), int(marker_2d[1]))

    return marker_2d

while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, arucoDict, parameters=arucoParams
    )
    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        total_markers = range(0, marker_IDs.size)

        #draw_height_cup_marker(frame, marker_corners, marker_IDs, cam_mat, dist_coef, tVec, 0)

        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv2.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            # Since there was mistake in calculating the distance approach point-outed in the Video Tutorial's comment
            # so I have rectified that mistake, I have test that out it increase the accuracy overall.
            # Calculating the distance
            distance = np.linalg.norm(tVec[i][0])
            #np.sqrt(
            #    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
            #)
            # Draw the pose of the marker
            #point = cv2.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            '''
            cv2.putText(
                frame,
                f"id: {ids[0]} Dist: {round(distance, 2)}",
                top_right,
                cv2.FONT_HERSHEY_PLAIN,
                1.3,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                bottom_right,
                cv2.FONT_HERSHEY_PLAIN,
                1.0,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            '''
            # print(ids, "  ", corners)
        
        markers_info = list(zip(marker_IDs, marker_corners, total_markers)) 
        cup_id = 0
        cup_marker = [m for m in markers_info if m[0] == cup_id]
        r = 32.5 # mm
        arm_marker_id=1
        cup_marker_id=0
        if cup_marker:
            cup_marker = cup_marker[0]
            cup_marker_3d_cam_tvec = tVec[cup_marker[2]][0]
            cup_marker_3d_cam_rvec = rVec[cup_marker[2]][0]
            cup_marker_2d = project3Dinto2D(cup_marker_3d_cam_tvec, cam_mat, dist_coef)
            for m in markers_info:
                if m[0] != cup_marker_id:
                    marker_3d_cam_t = tVec[m[2]][0]
                    marker_3d_cam_r = rVec[m[2]][0]
                    marker_2d = project3Dinto2D(marker_3d_cam_t, cam_mat, dist_coef)

                    # We get the 3D coordinates of the cup in the marker reference system
                    cup_3d_marker = cam_to_marker_coords(marker_3d_cam_r, marker_3d_cam_t, cup_marker_3d_cam_tvec)

                    #cup_center_marker = get_cup_center(cup_3d_marker, r)
                    ## Get center of the cup in the camera reference system
                    #cup_center_cam = get_cup_center(cup_marker_3d_cam_tvec, cup_marker_3d_cam_rvec, r)

                    #cup_center_marker = cam_to_marker_coords(marker_3d_cam_r, marker_3d_cam_t, cup_center_cam)

                    # Draw the center of the cup
                    cv2.circle(frame, cup_marker_2d, 5, (255, 0, 0), -1)

                    #center_cam_coords = marker_to_cam_coords(marker_3d_cam_r, marker_3d_cam_t, cup_center_marker)
                    #center_2d = project3Dinto2D(cup_center_cam, cam_mat, dist_coef)
                    center_2d = project3Dinto2D(cup_marker_3d_cam_tvec, cam_mat, dist_coef)
                    cv2.circle(frame, center_2d, 5, (0, 0, 255), -1)
                    cv2.circle(frame, marker_2d, 5, (0, 255, 0), -1)
                    cv2.circle(frame, cup_marker_2d, 5, (255, 0, 0), -1)
                    cv2.line(frame, marker_2d, center_2d, (255, 0, 0), 2)

                    if m[0] == arm_marker_id:
                        print(cup_3d_marker)
                        cup_3d_marker[1] = 0
                        arm_cup_distance = np.linalg.norm(marker_3d_cam_t - cup_marker_3d_cam_tvec) #np.linalg.norm(cup_3d_marker) #np.linalg.norm(marker_3d_cam_t - cup_center_cam)
                        arm_3d_pos = marker_3d_cam_t
                        #angle = np.arctan2(center_2d[1] - marker_2d[1], center_2d[0] - marker_2d[0])
                        director_vector = np.zeros(3)
                        director_vector[0] = 1
                        angle = get_marker_cup_angle(director_vector, cup_3d_marker)

                        # draw marker id
                        #cv2.putText(frame, str(arm_marker_id), (int(marker_2d[0]), int(marker_2d[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                        #cv2.putText(frame, str(cup_marker_id), (int(cup_marker_2d[0]), int(cup_marker_2d[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

                        # draw director vector
                        director_vector_3d = marker_to_cam_coords(marker_3d_cam_r, marker_3d_cam_t, director_vector)
                        director_vector_2d = project3Dinto2D(np.array([director_vector_3d[0]*1.5, director_vector_3d[1], director_vector_3d[2]]), cam_mat, dist_coef)
                        cv2.line(frame, marker_2d, director_vector_2d, (0, 255, 0), 2)
                        #cv2.line(frame, marker_2d, project3Dinto2D(np.array([0, 1.5, 0]), cam_mat, dist_coef), (255, 0, 0), 2)

                        # draw z axis
                        z_axis_3d = marker_to_cam_coords(marker_3d_cam_r, marker_3d_cam_t, np.array([0, 0, 1]))
                        z_axis_2d = project3Dinto2D(z_axis_3d, cam_mat, dist_coef)
                        cv2.line(frame, marker_2d, z_axis_2d, (255, 0, 0), 2)

                        # draw angle
                        cv2.putText(frame, str(round(np.rad2deg(angle),2)), (int(marker_2d[0]), int(marker_2d[1]) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

                        # Paint angle curve interior area
                        cv2.ellipse(frame, marker_2d, (100, 100), 0, 180, 180+np.rad2deg(angle), (0, 0, 255), 0)
                        print("[INFO] Angle between arm and cup: ", np.rad2deg(angle), " degrees")

                        # Draw arm-cup distance at middle of the line
                        cv2.putText(frame, str(round(arm_cup_distance,2)/10) + " cm", (int((center_2d[0] + marker_2d[0])/2), int((center_2d[1] + marker_2d[1])/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)    

                        print("[INFO] Distance between arm and cup: ", arm_cup_distance/10, " cm")







        # if cup_marker:
        #     print(cup_marker)
        #     cup_marker = cup_marker[0]
        #     cup_marker_3d_cam = tVec[cup_marker[2]][0]
        #     cup_marker_2d = project3Dinto2D(cup_marker_3d_cam, cam_mat, dist_coef)
        #     for m in markers_info:
        #         if m[0] != cup_id:
        #             marker_3d_cam_t = tVec[m[2]][0]
        #             marker_3d_cam_r = rVec[m[2]][0]
        #             marker_2d = project3Dinto2D(marker_3d_cam_t, cam_mat, dist_coef)
        #             cup_3d_marker = cam_to_marker_coords(marker_3d_cam_r, marker_3d_cam_t, cup_marker_3d_cam)

        #             cup_center_marker = get_cup_center(cup_3d_marker, r)
        #             center_cam_coords = marker_to_cam_coords(marker_3d_cam_r, marker_3d_cam_t, cup_center_marker)
        #             center_2d = project3Dinto2D(center_cam_coords, cam_mat, dist_coef)
        #             cv2.circle(frame, center_2d, 5, (0, 0, 255), -1)
        #             cv2.circle(frame, marker_2d, 5, (0, 255, 0), -1)
        #             cv2.circle(frame, cup_marker_2d, 5, (255, 0, 0), -1)
        #             cv2.line(frame, marker_2d, center_2d, (255, 0, 0), 2)
                    #cv2.line(frame, cup_marker_2d, center_2d, (255, 0, 0), 2)



        # for each pair of marker, calculate the distance between them
        # markers_info = list(zip(marker_IDs, marker_corners, total_markers))        
        # all_pairs = [(markers_info[i], markers_info[j]) for i in range(len(markers_info)) for j in range(i+1, len(markers_info))]
        # for m1, m2 in all_pairs:
        #   # Convert the image coordinates to integer values
        #   marker1_3d = tVec[m1[2]][0]
        #   marker2_3d = tVec[m2[2]][0]
          
        #   marker1_2d = projectMarker(marker1_3d, cam_mat, dist_coef)#(-int(tVec[m1[2]][0][0]), -int(tVec[m1[2]][0][1]))
        #   marker2_2d = projectMarker(marker2_3d, cam_mat, dist_coef)#(-int(tVec[m2[2]][0][0]), -int(tVec[m2[2]][0][1]))
        #   # Draw the line between the two markers
        #   color = (0, 0, 255)  # BGR color format (red in this case)
        #   thickness = 2
        #   cv2.line(frame, marker1_2d, marker2_2d, color, thickness)


    #frame, cup_3d_pos, arm_3d_pos, arm_cup_distance, angle = pose_estimation(frame, getArucoTag(args["type"]), cam_mat, dist_coef, arm_marker_id, cup_marker_id)

            
    cv2.imshow("frame", frame)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break
    
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
cap.release()
cv2.destroyAllWindows()
print("[INFO] Average FPS:", sum(fps_list)/len(fps_list))