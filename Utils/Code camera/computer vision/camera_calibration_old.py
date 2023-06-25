""" How to run the snippet:
python camera_calibration.py """

import numpy as np
import cv2
import glob

################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

chessboardSize = (7,7)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 45
objp = objp * size_of_chessboard_squares_mm

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


images = glob.glob('calibration/*.jpg')
images_calibrate = [img for img in images if 'calibrated' not in img]
for image in images_calibrate:
    print('[INFO] Reading ', image, '...')
    img = cv2.imread(image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

    # If found, add object points, image points (after refining them)
    if ret == True:

        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(1000)

        print('[INFO] Image sucessfully read')

        # Saving calibrated image
        cv2.imwrite(filename=image[:-4] + '_calibrated.jpg', img=img)

    else:
        print('[INFO] Fail when reading image')

cv2.destroyAllWindows()

############## CALIBRATION #######################################################
ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("[DATA] Camera matrix : \n")
print(cameraMatrix, "\n")
np.save("calibration/cameraMatrix.npy", cameraMatrix)

print("[DATA] dist : \n")
print(dist, "\n")
np.save("calibration/dist.npy", dist)

print("[DATA] rvecs : \n")
print(rvecs, "\n")
np.save("calibration/rvecs.npy", rvecs)

print("[DATA] tvecs : \n")
print(tvecs, "\n")
np.save("calibration/tvecs.npy", tvecs)