import cv2
import sys
import numpy as np

def getArucoTag(tag_type):

    """
    Parameters:
        - tag_type: string, type of the ArUco marker.
	
    Returns:
        - int, corresponding element of the set of markers.
    """

    # define names of each possible ArUco tag OpenCV supports
    aruco_dict = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }

    # verify that the supplied ArUCo tag exists and is supported by OpenCV
    if aruco_dict.get(tag_type, None) is None:
        print("[INFO] ArUCo tag of '{}' is not supported".format(tag_type))
        sys.exit(0)

    return aruco_dict[tag_type]

def yolo_pose_estimation(frame, x_mid, y_mid, intrinsic_camera, distortion, cup_depth, marker_rvec, marker_tvec):
    cup_3d_position_cam = project2Dinto3D((x_mid, y_mid), intrinsic_camera, distortion)
    cup_3d_position_cam[2] = cup_depth

    # draw 3D position of the cup
    cup_2d = project3Dinto2D(cup_3d_position_cam, intrinsic_camera, distortion)
    print("CUP 2D", cup_2d)
    cv2.circle(frame, (int(cup_2d[0]), int(cup_2d[1])), 5, (0, 255, 0), -1)

    # Change to marker coords
    cup_3d_position_marker = cam_to_marker_coords(marker_rvec, marker_tvec, cup_3d_position_cam)

    
    print("*** Cup 3d position 2: ", cup_3d_position_marker)
    return frame

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, arm_marker_id, cup_marker_id, x_mid, y_mid, cup_depth):
	
    """
    Parameters:
        - frame: array, frame of the image to be analyzed.
        - aruco_dict_type: string, type of the ArUco marker.
        - matrix_coefficients: array, 3x3 matrix containing the camera instrinsic parameters.
        - distortion_coefficients: array, 1x5 vector containing the parameters of lens distortion effect.
	
    Returns:
        - frame, with the added pose estimators.
    """
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
	# get ArUco dictionary and parameters
    arucoDict = cv2.aruco.getPredefinedDictionary(aruco_dict_type) # old Dictionary_get
    arucoParams = cv2.aruco.DetectorParameters() # old DetectorParameters_create

	# detecting ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
    arm_cup_distance = None
    arm_3d_pos = None
    cup_center_cam = None
    angle = None
    if len(corners) > 0:
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(frame, corners) 
        
        # Calculate pose of markers
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 2.3, matrix_coefficients, distortion_coefficients)
 
        for i in range(len(ids)):
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvecs[i], tvecs[i], 0.01) 

            # Average rvec and tvec values for all corners of the marker
            rvec_avg = np.mean(rvecs[i], axis=0)
            tvec_avg = np.mean(tvecs[i], axis=0)

            # Compute center position of the marker
            marker_center_3d_position = -np.transpose(cv2.Rodrigues(rvec_avg)[0]) @ tvec_avg
            #print("*** Tag 3d position: ", marker_center_3d_position)

        #draw_distances_markers(frame, corners, ids, matrix_coefficients, distortion_coefficients, tvecs)
        #draw_distances_markers_cup(frame, cup_3d_position, corners, ids, matrix_coefficients, distortion_coefficients, tvecs)
        #draw_height_cup_marker(frame, corners, ids, matrix_coefficients, distortion_coefficients, tvecs, 0)
        total_markers = range(0, ids.size)
        markers_info = list(zip(ids, corners, total_markers)) 
        cup_marker = [m for m in markers_info if m[0] == cup_marker_id]
        r = 6.6 # cm
        if cup_marker:
            print(cup_marker)
            cup_marker = cup_marker[0]
            cup_marker_3d_cam_tvec = tvecs[cup_marker[2]][0]
            cup_marker_3d_cam_rvec = rvecs[cup_marker[2]][0]
            cup_marker_2d = project3Dinto2D(cup_marker_3d_cam_tvec, matrix_coefficients, distortion_coefficients)
            for m in markers_info:
                if m[0] != cup_marker_id:
                    marker_3d_cam_t = tvecs[m[2]][0]
                    marker_3d_cam_r = rvecs[m[2]][0]
                    marker_2d = project3Dinto2D(marker_3d_cam_t, matrix_coefficients, distortion_coefficients)

                    # We get the 3D coordinates of the cup in the marker reference system
                    cup_3d_marker = cam_to_marker_coords(marker_3d_cam_r, marker_3d_cam_t, cup_marker_3d_cam_tvec)

                    #cup_center_marker = get_cup_center(cup_3d_marker, r)
                    cup_center_cam = get_cup_center(cup_marker_3d_cam_tvec, cup_marker_3d_cam_rvec, r)
                    #center_cam_coords = marker_to_cam_coords(marker_3d_cam_r, marker_3d_cam_t, cup_center_marker)
                    center_2d = project3Dinto2D(cup_center_cam, matrix_coefficients, distortion_coefficients)
                    cv2.circle(frame, center_2d, 5, (0, 0, 255), -1)
                    cv2.circle(frame, marker_2d, 5, (0, 255, 0), -1)
                    cv2.circle(frame, cup_marker_2d, 5, (255, 0, 0), -1)
                    cv2.line(frame, marker_2d, center_2d, (255, 0, 0), 2)

                    if m[0] == arm_marker_id:
                        arm_cup_distance = np.linalg.norm(marker_3d_cam_t - cup_center_cam)
                        arm_3d_pos = marker_3d_cam_t
                        #angle = np.arctan2(center_2d[1] - marker_2d[1], center_2d[0] - marker_2d[0])
                        director_vector = np.zeros(3)
                        director_vector[0] = 1
                        angle = get_marker_cup_angle(director_vector, cup_3d_marker)

                        # draw marker id
                        #cv2.putText(frame, str(arm_marker_id), (int(marker_2d[0]), int(marker_2d[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                        #cv2.putText(frame, str(cup_marker_id), (int(cup_marker_2d[0]), int(cup_marker_2d[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                        frame = yolo_pose_estimation(frame, x_mid, y_mid, matrix_coefficients, distortion_coefficients, cup_depth, marker_3d_cam_r, marker_3d_cam_t)
                        # draw director vector
                        director_vector_3d = marker_to_cam_coords(marker_3d_cam_r, marker_3d_cam_t, director_vector)
                        director_vector_2d = project3Dinto2D(director_vector_3d, matrix_coefficients, distortion_coefficients)
                        cv2.line(frame, marker_2d, director_vector_2d, (0, 0, 255), 2)

                        # draw z axis
                        z_axis_3d = marker_to_cam_coords(marker_3d_cam_r, marker_3d_cam_t, np.array([0, 0, 1]))
                        z_axis_2d = project3Dinto2D(z_axis_3d, matrix_coefficients, distortion_coefficients)
                        cv2.line(frame, marker_2d, z_axis_2d, (255, 0, 0), 2)

                        # draw angle
                        #cv2.putText(frame, str(np.rad2deg(angle)), (int(marker_2d[0]), int(marker_2d[1]) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

                        # Paint angle curve interior area
                        cv2.ellipse(frame, marker_2d, (100, 100), 0, 0, np.rad2deg(angle), (0, 0, 255), -1)

                        print("[INFO] Angle between arm and cup: ", np.rad2deg(angle), " degrees")




    return (frame, cup_center_cam, arm_3d_pos, arm_cup_distance, angle)

def project3Dinto2D(marker_3d, cam_mat, dist_coef):
    '''Project a 3D Aruco marker onto the image plane
    :param marker_3d: 3D position of the Aruco marker
    :param cam_mat: camera matrix
    :param dist_coef: distortion coefficients
    :return: 2D position of the Aruco marker
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

def project2Dinto3D(point_2d, intrinsic_camera, distortion):
    '''Project a 2D point into 3D space
    :param point_2d: 2D point in image plane
    :param intrinsic_camera: intrinsic camera matrix
    :param distortion: distortion coefficients
    :return: 3D point in camera coordinate system
    '''
    # Undistort the point (optional if you have already undistorted the image)
    point_2d_undistorted = cv2.undistortPoints(point_2d, intrinsic_camera, distortion)

    # Add depth information (z-coordinate)
    point_3d_homogeneous = np.append(point_2d_undistorted, np.ones((1, 1)))#np.hstack((point_2d_undistorted, np.array([[1]])))

    # Calculate the inverse of the intrinsic matrix
    intrinsic_camera_inv = np.linalg.inv(intrinsic_camera)

    # Perform back-projection from 2D to 3D
    point_3d_homogeneous = np.dot(intrinsic_camera_inv, point_3d_homogeneous)

    # Normalize the 3D point by dividing by the last coordinate
    point_3d = point_3d_homogeneous[:3]#/ point_3d_homogeneous[3]

    return point_3d

def draw_distances_markers_cup(frame, cup_3d_position, corners, marker_IDs, matrix_coefficients, distortion_coefficients, tVec):
    '''Draw the distance between the cup and the markers
    :param frame: The frame to draw on
    :param cup_3d_position: The 3D position of the cup
    :param corners: The corners of the markers
    :param marker_IDs: The IDs of the markers
    :param matrix_coefficients: The camera matrix
    :param distortion_coefficients: The distortion coefficients
    :param tVec: The translation vectors of the markers
    :return: None
    '''

    total_markers = range(0, marker_IDs.size)
    #cup_3d = project2Dinto3D(cup, matrix_coefficients, distortion_coefficients)
    for m_id, corners, i in zip(marker_IDs, corners, total_markers):
        marker_3d = tVec[i][0]

        # Calculate the Euclidean distance between the two markers in 3D space
        distance = np.linalg.norm(marker_3d - cup_3d_position)

        # Convert the distance to a string with two decimal places
        distance_str = "{:.2f}".format(distance)

        # Define the label text
        label_text = f"Distance: {distance_str} units"

        marker_2d = project3Dinto2D(marker_3d, matrix_coefficients, distortion_coefficients)
        cup_2d = project3Dinto2D(cup_3d_position, matrix_coefficients, distortion_coefficients)

        # Draw the line between the two markers
        color = (255, 0, 0)  # BGR color format (red in this case)
        thickness = 2
        cup_int = (int(cup_2d[0]), int(cup_2d[1]))
        cv2.line(frame, marker_2d, cup_int, color, thickness)

        label_position = ((marker_2d[0] + cup_int[0]) // 2, (marker_2d[1] + cup_int[1]) // 2 + 20)

        # Draw the label on the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (255, 0, 0)  # BGR color format (red in this case)
        thickness = 3
        cv2.putText(frame, label_text, label_position, font, font_scale, color, thickness)

        # Draw a line from the center of the marker to z=cup_3d_position[z]
        height_cup = cup_3d_position[2]
        vector_from_marker = np.array([marker_3d[0], marker_3d[1], height_cup])
        vector_from_marker_2d = project3Dinto2D(vector_from_marker, matrix_coefficients, distortion_coefficients)
        cv2.line(frame, marker_2d, vector_from_marker_2d, color, thickness)
        color = (255, 0, 0)  # BGR color format (red in this case)
        thickness = 2
        cv2.line(frame, vector_from_marker_2d, cup_2d, color, thickness)

def draw_height_cup_marker(frame, corners, marker_IDs, matrix_coefficients, distortion_coefficients, tVec, cup_marker_id):
    '''Draw the height of the cup from the marker
    :param frame: image
    :param corners: corners of the markers
    :param marker_IDs: IDs of the markers
    :param matrix_coefficients: intrinsic camera matrix
    :param distortion_coefficients: distortion coefficients
    :param tVec: translation vectors
    :param cup_marker_id: ID of the cup marker
    :return: None
    '''
    total_markers = range(0, marker_IDs.size)
    markers_info = list(zip(marker_IDs, corners, total_markers))
    cup_marker = [m for m in markers_info if m[0] == cup_marker_id]
    if len(cup_marker) == 0:
        print("[ERROR] Cup marker not found")
        return
    cup_marker = cup_marker[0]
    cup_marker_3d = tVec[cup_marker[2]][0]
    cup_marker_2d = project3Dinto2D(cup_marker_3d, matrix_coefficients, distortion_coefficients)
    for m in markers_info:
        if m[0] != cup_marker_id:
            marker_3d = tVec[m[2]][0]
            marker_2d = project3Dinto2D(marker_3d, matrix_coefficients, distortion_coefficients)
            # Draw the line between the two markers
            color = (255, 0, 0)
            thickness = 2
            cv2.line(frame, marker_2d, cup_marker_2d, color, thickness)
            # Draw a line from the center of the marker to z=cup_3d_position[z]
            height_cup = cup_marker_3d[1]
            vector_from_marker = np.array([marker_3d[0], height_cup, marker_3d[2]])
            vector_from_marker_2d = project3Dinto2D(vector_from_marker, matrix_coefficients, distortion_coefficients)
            color = (0, 0, 255)  # RGB color format (blue in this case)
            thickness = 2
            cv2.line(frame, marker_2d, vector_from_marker_2d, color, thickness)
            #cv2.line(frame, vector_from_marker_2d, cup_marker_2d, color, thickness)


def draw_distances_markers(frame, corners, marker_IDs, matrix_coefficients, distortion_coefficients, tVec):
    '''Draw the distance between all the markers in the image
    :param frame: The image to draw on
    :param corners: The corners of the markers in the image
    :param marker_IDs: The IDs of the markers in the image
    :param matrix_coefficients: The camera matrix
    :param distortion_coefficients: The distortion coefficients
    :param tVec: The translation vectors of the markers
    
    :return: None
    '''
    total_markers = range(0, marker_IDs.size)
    markers_info = list(zip(marker_IDs, corners, total_markers))
    all_pairs = [(markers_info[i], markers_info[j]) for i in range(len(markers_info)) for j in range(i+1, len(markers_info))]
    for m1, m2 in all_pairs:
        
        # Convert the image coordinates to integer values
        marker1_3d = tVec[m1[2]][0]
        marker2_3d = tVec[m2[2]][0]

        # Calculate the Euclidean distance between the two markers in 3D space
        distance = np.linalg.norm(marker1_3d - marker2_3d)

        # Convert the distance to a string with two decimal places
        distance_str = "{:.2f}".format(distance)

        # Define the label text
        label_text = f"Distance: {distance_str} units"
        
        marker1_2d = project3Dinto2D(marker1_3d, matrix_coefficients, distortion_coefficients)#(-int(tVec[m1[2]][0][0]), -int(tVec[m1[2]][0][1]))
        marker2_2d = project3Dinto2D(marker2_3d, matrix_coefficients, distortion_coefficients)#(-int(tVec[m2[2]][0][0]), -int(tVec[m2[2]][0][1]))

        # Draw the line between the two markers
        color = (0, 0, 255)  # BGR color format (red in this case)
        thickness = 2
        cv2.line(frame, marker1_2d, marker2_2d, color, thickness)

        label_position = ((marker1_2d[0] + marker2_2d[0]) // 2, (marker1_2d[1] + marker2_2d[1]) // 2 + 20)

        # Draw the label on the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (0, 0, 255)  # BGR color format (red in this case)
        thickness = 3
        cv2.putText(frame, label_text, label_position, font, font_scale, color, thickness)



def inversePerspective(rvec, tvec):
    '''Calculate the inverse perspective of a given rotation and translation vector
    :param rvec: the rotation vector
    :param tvec: the translation vector

    :return: the inverse rotation and translation vector
    '''
    R, _ = cv2.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(-R, np.matrix(tvec))
    invRvec, _ = cv2.Rodrigues(R)
    return invRvec, invTvec

def estimate_cup_depth(width_px, height_px, real_width_cm, real_height_cm, intrinsic_camera):
    '''Estimate the depth of the cup relative to the camera
    :param width_px: the width of the cup's bounding box in pixels
    :param height_px: the height of the cup's bounding box in pixels
    :param real_width_cm: the width of the cup in the real world in centimeters
    :param real_height_cm: the height of the cup in the real world in centimeters
    :param intrinsic_camera: the intrinsic camera matrix

    :return: the estimated depth of the cup in centimeters
    '''
    print("[INFO] estimating cup depth...")

    # Calculate the cup's size in the undistorted image
    cup_size_pixels = max(width_px, height_px)

    # Calculate the cup's size in the real world
    cup_size_cm = max(real_width_cm, real_height_cm)

    scaling_factor = cup_size_cm / cup_size_pixels

    # Estimate the depth of the cup relative to the camera
    depth = scaling_factor * intrinsic_camera[0, 0]  # Assuming a planar scene

    return depth

def estimate_cup_3d_position_ai(x_center, y_center, depth, intrinsic_camera):
    '''Estimate the 3D position of the cup's center in the camera's coordinate system
    Uses information from the cup's bounding box and the estimated depth of the cup
    :param x_center: the x coordinate of the cup's bbox center in the image
    :param y_center: the y coordinate of the cup's bbox center in the image
    :param depth: the estimated depth of the cup in centimeters
    :param intrinsic_camera: the camera's intrinsic matrix

    :return: the 3D position of the cup's center in the camera's coordinate system
    '''

    print("[INFO] estimating cup position...")
    # Get the focal length and principal point from the intrinsic matrix
    focal_length = intrinsic_camera[0, 0]
    principal_point_x = intrinsic_camera[0, 2]
    principal_point_y = intrinsic_camera[1, 2]

    x_ndc = (x_center - principal_point_x) / focal_length
    y_ndc = (y_center - principal_point_y) / focal_length

    x = x_ndc * depth
    y = y_ndc * depth
    z = depth

    return np.array([x, y, z])

def get_cup_3d_position(x, y, cup_width_px):

    print("[INFO] retrieving cup position...")

    # Define the real width of the cup in centimeters
    cup_width_cm = 8.7

    # Define the 2D position of the cup's center
    cup_center_2d = (x, y)  # Replace with the actual coordinates

    # Load the calibration data from file
    intrinsic_camera = np.load("calibration/cameraMatrix.npy")

    # Get the focal length and principal point from the intrinsic matrix
    fx = intrinsic_camera[0, 0]
    fy = intrinsic_camera[1, 1]
    cx = intrinsic_camera[0, 2]
    cy = intrinsic_camera[1, 2]

    # Get the camera position from the rotation and translation vectors
    rvecs = np.load("calibration/rvecs.npy")
    tvecs = np.load("calibration/tvecs.npy")

    # loop over each rotation vector and convert to rotation matrix
    R_list = [cv2.Rodrigues(vec)[0] for vec in rvecs]

    # concatenate rotation matrices along third axis to get 3D array
    R_array = np.dstack(R_list)

    # get average rotation vector
    R = np.mean(R_array, axis=2)

    # get average translation vector
    T = np.mean(tvecs.squeeze(), axis=0)

    camera_position = -R.T @ T

    # Define the calibration matrix of the camera
    K = np.array([[fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]])

    # Define the 3D position of the cup's center in the camera coordinate system
    cup_center_3d_cam = np.array([cup_center_2d[0], cup_center_2d[1], 1])

    # Apply the inverse of the calibration matrix to the 3D position
    cup_center_3d_cam = np.linalg.inv(K) @ cup_center_3d_cam

    # Normalize the 3D position to get a direction vector
    cup_direction_cam = cup_center_3d_cam / np.linalg.norm(cup_center_3d_cam)

    # Calculate the distance to the cup from the camera
    cup_distance_cm = (cup_width_cm / 2) / np.tan(cup_width_px / (2 * fx))

    # Calculate the 3D position of the cup's center in the world coordinate system
    cup_center_3d_world = camera_position + cup_direction_cam * cup_distance_cm

    return cup_center_3d_world

def undistort_image(image, cameraMatrix, distCoeffs):
    '''Undistort an image using the camera matrix and distortion coefficients
    :param image: the image to undistort
    :param cameraMatrix: the camera matrix
    :param distCoeffs: the distortion coefficients

    :return: the undistorted image
    '''
    #load image image = cv2.imread(fileName) #images = glob.glob(pathName + '*.jpg') #loop within a specified directory #for fileName in images: #image = cv2.imread(fileName)

    #set camera parameters 
    height, width = image.shape[:2] 

    #create new camera matrix 
    newCameraMatrix, validPixROI = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs,(width, height), 1, (width, height))

    #undistort 
    outputImage = cv2.undistort(image, cameraMatrix, distCoeffs, None, newCameraMatrix)

    #crop, modified 
    x, y, w, h = validPixROI #(211, 991, 547, 755) 
    outputImage = outputImage[y:y+h, x:x+w]#outputImage[y-200:y+h+200, x-40:x+w+80] #fudge factor to minimize cropping

    return outputImage

def cam_to_marker_coords(r_vec_origin, t_vec_origin, t_vec_cam):
    '''Converts a vector in camera coordinates to a vector in marker coordinates.
    :param r_vec_origin: rotation vector of the marker
    :param t_vec_origin: translation vector of the marker
    :param t_vec_cam: translation vector of the camera
    :return: t_vec_marker: translation vector of the camera in marker coordinates
    '''
    matrix_marker_to_cam = np.zeros((4, 4))
    matrix_marker_to_cam[3, 3] = 1
    matrix_marker_to_cam[0:3, 0:3], _ = cv2.Rodrigues(r_vec_origin)
    matrix_marker_to_cam[0:3, 3] = t_vec_origin

    matrix_cam_to_marker = np.linalg.inv(matrix_marker_to_cam)
    t_vec_marker = matrix_cam_to_marker @ np.append(t_vec_cam, 1)

    return t_vec_marker[0:3]

def marker_to_cam_coords(r_vec_origin, t_vec_origin, t_vec_marker):
    '''Converts a vector in marker coordinates to a vector in camera coordinates.
    :param r_vec_origin: rotation vector of the marker
    :param t_vec_origin: translation vector of the marker
    :param t_vec_marker: a translation vector in marker coordinates
    :return: t_vec_cam: translation vector of the camera
    '''
    matrix_marker_to_cam = np.zeros((4, 4))
    matrix_marker_to_cam[3, 3] = 1
    matrix_marker_to_cam[0:3, 0:3], _ = cv2.Rodrigues(r_vec_origin)
    matrix_marker_to_cam[0:3, 3] = t_vec_origin

    t_vec_cam = matrix_marker_to_cam @ np.append(t_vec_marker, 1)

    return t_vec_cam[0:3]


def get_cup_center(cup_cam_tvec, cup_cam_rvec, r):
    '''Calculates the center of the cup in cam coordinates.
    It assumes the cup center is at a -r distance from the marker in the X direction.
    :param cup_marker_pos: position of the cup in marker coordinates
    :param r: radius of the cup
    '''

    cup_center = np.zeros(3)
    cup_center[0] = 0
    cup_center[1] = 0
    cup_center[2] = -r

    cup_center = marker_to_cam_coords(cup_cam_rvec, cup_cam_tvec, cup_center)

    return cup_center

def get_marker_cup_angle(marker_3d_pos, cup_3d_pos):
    '''Calculates the angle between the cup and the arm marker in marker coordinates.
    It assumes the arm marker is in the same (XY) plane as the cup.
    :param cup_3d_pos: position of the cup in marker coordinates
    :param marker_3d_pos: position of the arm marker in marker coordinates
    '''

    cup_3d_pos[2] = 0
    marker_3d_pos[2] = 0
    cup_3d_pos = cup_3d_pos / np.linalg.norm(cup_3d_pos)
    marker_3d_pos = marker_3d_pos / np.linalg.norm(marker_3d_pos)
    angle = np.arccos(np.dot(marker_3d_pos, cup_3d_pos))
    #angle = np.arccos(np.dot(cup_3d_pos, marker_3d_pos))

    return angle