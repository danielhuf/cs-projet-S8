import numpy as np
import cv2
import open3d as o3d
from frame_change import *
from pcd_processing import *
from image_processing import *
import time

WIDTH = 480
LENGTH = 640

corners_world = chessboard_corner_position_world(
    np.array([0, 0, 0]), 10, 7, 0.025, 0.025)  # Generate real position

# not sure
camera_matrix = np.array([[576.2010093732916, 0, 333.7638119169971], [
    0, 575.8066271149149, 239.9059593076319], [0, 0, 1]])


def calibration(img_path, pcd_path, method='svd'):
    """Calculate the frame change between camera and world
    """
    global corners_world
    # Read file
    img = cv2.imread(img_path)
    pcd = readTXT(pcd_path)

    # Using opencv to find chessboard corner in 2D image
    corners_img = find_chessboard_corners(img, 10, 7)
    print(corners_img)
    if len(corners_img) == 0:
        return False, None, None

    if method == 'svd':
        # Using SVD method to solve transformation matrix
        _, corners_camera, corners_world = chessboard_corner_2DT3D(
            pcd, corners_img, corners_world)
        # print(len(corners_world))
        rotation, translation = frame_change_svd(corners_world, corners_camera)
        # print(rotation, transition)
    elif method == 'PnP':
        rotation, translation = frame_chage_PnP(
            corners_world, corners_img, camera_matrix)
    else:
        print("Error please select an existing method to solve transformation matrix")
        return False, None, None

    return True, rotation, translation


def find_work_space(img_path):

    cv2.namedWindow("TrackBars")
    cv2.resizeWindow("TrackBars", 640, 340)
    # Create track bars
    cv2.createTrackbar("Left top u", "TrackBars", 0, LENGTH, empty)
    cv2.createTrackbar("Left top v", "TrackBars", 0, WIDTH, empty)
    cv2.createTrackbar("Length of the window", "TrackBars", 50, LENGTH, empty)
    cv2.createTrackbar("Width of the window", "TrackBars", 50, WIDTH, empty)

    while True:
        # load image
        img = cv2.imread(img_path)

        # get trackbar values
        left_u = cv2.getTrackbarPos("Left top u", "TrackBars")
        left_v = cv2.getTrackbarPos("Left top v", "TrackBars")
        length = cv2.getTrackbarPos("Length of the window", "TrackBars")
        width = cv2.getTrackbarPos("Width of the window", "TrackBars")
        right_u = left_u+length
        right_v = left_v+width

        img = cv2.rectangle(img, (left_u, left_v),
                            (right_u, right_v), (0, 255, 0))
        cv2.imshow("Origin", img)

        img_seg = img[left_v:right_v, left_u:right_u]
        cv2.imshow("Segment", img_seg)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

    return left_u, left_v, length, width


def get_robot_command(posture_chessboard, object):
    ##### Define transformation from chessboard to robot #########
    dx = 0.194
    dy = -0.105
    dz = 0
    T_robot = np.array([[0, -1, 0, dx],
                        [1, 0, 0, dy],
                        [0, 0, 1, dz],
                        [0, 0, 0, 1]])

    posture_robot = np.dot(T_robot, posture_chessboard)
    x_orientation = posture_robot[:3, 0]
    y_orientation = posture_robot[:3, 1]
    z_orientation = posture_robot[:3, 2]
    position = posture_robot[:3, 3]  # x,y,z
    robot_orient = np.arctan(position[1]/position[0])
    # the angle between two z axes
    gripper_orient = np.arccos(np.abs(z_orientation[2]))

    if np.abs(gripper_orient) < np.pi/12:  # verticle
        if object == 'cube':
            # first: ok if can be picked up, second: 3D position, third: 90 means gripper verticle, 0 horizontal
            return True, position, 90
        else:
            object_offset = np.arccos(np.abs(np.dot(y_orientation, np.array(
                [np.cos(robot_orient), np.sin(robot_orient), 0]))))
            if object_offset < np.pi/6:
                return True, position, 90
            else:
                return False, None, None
    else:  # horizontal
        if object == 'triangularPrism':
            normal = np.cross(y_orientation, z_orientation)
            if np.arccos(np.abs(np.dot(normal, np.array([np.sin(robot_orient), np.cos(robot_orient), 0])))) < np.pi/6:
                return True, position, 0
            else:
                return False, None, None
        else:
            return True, position, 0


def initialize_camera(img_path, pcd_path):
    img_path = './kinect/registeredRgbImg_t.jpg'
    pcd_path = './kinect/test_pcd_t.txt'

    # calibrate camera
    ok = False
    frame_R, frame_T = np.ones((3, 3)), np.zeros((1, 3))
    while not ok:
        ok, frame_R, frame_T = calibration(
            img_path, pcd_path, method="svd")
    frame_transformation = np.hstack(
        (np.transpose(frame_R), -np.dot(np.transpose(frame_R), frame_T.reshape(3, 1))))
    frame_transformation = np.vstack(
        (frame_transformation, np.array([0, 0, 0, 1])))
    print("Transformation matrix calculated:\n", frame_transformation)
    np.save("frame_transformation.npy", frame_transformation)

    # find work space
    left_u, left_v, length, width = find_work_space(img_path)
    segmentation = np.array([left_u, left_v, length, width])
    np.save("segmentation.npy", segmentation)


def get_object_posture(img_path, pcd_path):
    frame_transformation = np.load("frame_transformation.npy")
    segmentation = np.load("segmentation.npy")
    left_u, left_v, length, width = segmentation
    # print(frame_transformation)

    #
    img_path = './kinect/registeredRgbImg.jpg'
    pcd_path = './kinect/test_pcd.txt'
    #
    while True:
        img = cv2.imread(img_path)
        pcd = readTXT(pcd_path)

        img = img[left_v:left_v+width, left_u:left_u+length]
        cv2.imshow('img', img)

        # select points corresponding to the work space
        x = np.arange(left_v, left_v+width)
        y = np.arange(left_u, left_u+length)
        ind_x, ind_y = np.meshgrid(x, y)
        ind_work_space = LENGTH * \
            np.reshape(np.transpose(ind_x), -1) + \
            np.reshape(np.transpose(ind_y), -1)
        pcd = select_pcd_by_index(pcd, ind_work_space)

        # Find objects to pick up
        mask, max_mask = select_color(img)
        cv2.imshow('max_mask', max_mask)

        # Process object one by one
        loc_i, loc_j = np.where(max_mask == 255)
        ind_object_points = loc_i*length+loc_j
        pcd_object = select_pcd_by_index(pcd, ind_object_points)

        # Transform object's position into world frame
        pcd_object = clear_invalide(pcd_object)
        pcd_object.transform(frame_transformation)
        pcd_filtered = pcd_filter_radius(pcd_object, 30, 0.03)

        # object recognizing and posture estimation
        posture, object = object_recognize_and_find_position(
            pcd_filtered, voxel_size=0.002)

        ok, position, gripper_orient = get_robot_command(posture, object)
        # # to control the process
        cv2.waitKey(0)
        # #

        if ok:
            return object, position, gripper_orient
        else:
            print(
                "[Error] Please check if there are still bricks remained or if the brick is correctly placed")
            print(
                "If error is fixed, please press any key to continue. If you want to stop, press 'q'")
            if cv2.waitKey(0) & 0xFF == ord('q'):
                return None, None, None


def main(img_path, pcd_path, start=1):
    print("####################### Program Start ######################\n Press 'q' to shut down program")
    # First calculate
    if start <= 1:
        #
        img_path = './kinect/registeredRgbImg1.jpg'
        pcd_path = './kinect/test_pcd1.txt'
        #
        ok = False
        frame_R, frame_T = np.ones((3, 3)), np.zeros((1, 3))
        while not ok:
            ok, frame_R, frame_T = calibration(
                img_path, pcd_path, method="svd")
        frame_transformation = np.hstack(
            (np.transpose(frame_R), -np.dot(np.transpose(frame_R), frame_T.reshape(3, 1))))
        frame_transformation = np.vstack(
            (frame_transformation, np.array([0, 0, 0, 1])))
        print("Transformation matrix calculated:\n", frame_transformation)
        np.save("frame_transformation.npy", frame_transformation)

    if start <= 2:
        #
        img_path = './kinect/registeredRgbImg2.jpg'
        #
        left_u, left_v, length, width = find_work_space(img_path)
        segmentation = np.array([left_u, left_v, length, width])
        np.save("segmentation.npy", segmentation)

    # Avoid calibration and segmentation every time
    if start <= 3:
        # Preparation
        frame_transformation = np.load("frame_transformation.npy")
        segmentation = np.load("segmentation.npy")
        left_u, left_v, length, width = segmentation
        # print(frame_transformation)

        #
        img_path = './kinect/registeredRgbImg2.jpg'
        pcd_path = './kinect/test_pcd2.txt'
        #
        while True:
            start = time.time()
            img = cv2.imread(img_path)
            # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            pcd = readTXT(pcd_path)

            img = img[left_v:left_v+width, left_u:left_u+length]
            cv2.imshow('img', img)

            x = np.arange(left_v, left_v+width)
            y = np.arange(left_u, left_u+length)
            ind_x, ind_y = np.meshgrid(x, y)
            ind_work_space = LENGTH * \
                np.reshape(np.transpose(ind_x), -1) + \
                np.reshape(np.transpose(ind_y), -1)
            pcd = select_pcd_by_index(pcd, ind_work_space)

            # Find objects to pick up
            mask, max_mask = select_color(img)
            cv2.imshow('max_mask', max_mask)
            cv2.imshow('mask', mask)

            # Process object one by one
            loc_i, loc_j = np.where(max_mask == 255)
            # loc_i, loc_j = np.where(mask == 255)
            ind_object_points = loc_i*length+loc_j
            pcd_object = select_pcd_by_index(pcd, ind_object_points)
            #
            # visualize_pcd(pcd_object)
            # pcd_object = clear_invalide(pcd_object)
            # pcd_filtered = pcd_filter_radius(pcd_object, 30, 0.03)
            # visualize_pcd(pcd_filtered)
            #

            # Transform object's position into world frame
            pcd_object = clear_invalide(pcd_object)
            pcd_object.transform(frame_transformation)
            pcd_filtered = pcd_filter_radius(pcd_object, 30, 0.03)

            # # object recognizing and posture estimation
            posture, object = object_recognize_and_find_position(
                pcd_filtered, voxel_size=0.002)
            end = time.time()

            print("Time: ", end-start)

            # print(T)
            y = posture[:3, 1]
            print("The orientation: ", np.arccos(
                abs(y[1]))/(np.pi)*180, "\nThe position: ", posture[:3, 3])

            ok, position, gripper_orient = get_robot_command(posture, object)
            print(ok, position)
            # break

            # press 'q' to shut down program
            if cv2.waitKey(0) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break


if __name__ == "__main__":
    img_path = ""
    pcd_path = ""
    # t1 = os.time()
    main(img_path, pcd_path, start=3)
    # t2 = os.time()
