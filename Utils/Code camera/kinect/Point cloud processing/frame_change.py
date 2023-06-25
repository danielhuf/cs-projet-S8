import cv2
import numpy as np


"""
This file contains the method to calculate the extrinsic matrix of camera i.e. the transformation matrix from robot frame to camera frame.
"""


colorImgPath = './Project1/colorImg.jpg'
depthImgPath = './depthImg.png'
registeredRgbImg = './registeredRgnImg.jpg'


def find_chessboard_corners(img, row, col):
    """ Find inner points in a chessboard. row and col variables are set with respect to inner points
    usually, the points are in such an order: first from top to bottom, then from right to left

    """
    size = (row, col)
    ok, corners = cv2.findChessboardCorners(img, size, None)
    cv2.drawChessboardCorners(img, size, corners, ok)

    if ok:
        # print(corners)
        cv2.imshow('img', img)
        cv2.waitKey(0)
        return corners
    else:
        print("Couldn't find corners")
        return None


def chessboard_corner_position_world(start_point, row, col, row_width, col_width=None):
    # For convenience, generate the world frame coordinates of corners used for test
    if col_width == None:
        col_width = row_width
    points = np.zeros((row*col, 3))
    for i in range(col):
        for j in range(row):
            points[i*row+j] = start_point+[j*row_width, i*col_width, 0]
    return points


def frame_chage_PnP(object_points, image_points, camera_matrix, distortion_coeffs=None):
    """ solve the transformation matrix with PnP method
    """
    success, rot_vector, trans_vector = cv2.solvePnP(
        object_points, image_points, camera_matrix, distortion_coeffs)
    if success:
        return cv2.Rodrigues(rot_vector)[0], trans_vector
    else:
        return None, None
        print("Cannot sovle the the transformation matrix")


def frame_change_svd(points1, points2):
    """ To calculate rotation and translation matrices between two frame using SVD decomposition. R,t s.t. p2=R*p1+t.

    points1: numpy array, first point set
    points2: numpy array, second point set
    """
    if points1.shape[0] != points2.shape[0]:
        print("Error, two sets have different dimensions!")

    centroid_A = np.mean(points1, axis=0)
    centroid_B = np.mean(points2, axis=0)

    H = np.dot((points1-centroid_A).T, points2-centroid_B)
    U, S, Vh = np.linalg.svd(H)

    R = np.dot(Vh.T, U.T)
    if np.linalg.det(R) < 0:
        Vh[2] = -Vh[2]
        R = np.dot(Vh.T, U.T)

    t = centroid_B.T-np.dot(R, centroid_A.T)

    return R, t


if __name__ == "__main__":
    color = cv2.imread(colorImgPath)
    corners = find_chessboard_corners(color, 7, 10)
    points = chessboard_corner_position_world(
        np.array([0, 0, 0]), 7, 10, 0.025)
    camera_matrix = np.array([[576.2010093732916, 0, 333.7638119169971], [
                             0, 575.8066271149149, 239.9059593076319], [0, 0, 1]])
    rot_matrix, trans_matrix = frame_chage_PnP(
        points, corners, camera_matrix)
    print(rot_matrix, trans_matrix)
