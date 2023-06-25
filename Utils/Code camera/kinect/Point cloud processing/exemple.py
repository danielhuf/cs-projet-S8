import numpy as np
import cv2
import open3d as o3d
from pcd_processing import *
from main import *

############ test cup ######################
model_path = './models/cup.pcd'
img_path = './kinect/registeredRgbImg2.jpg'
pcd_path = './kinect/test_pcd2.txt'

#
left_u, left_v, length, width = find_work_space(img_path)
segmentation = np.array([left_u, left_v, length, width])
np.save("segmentation.npy", segmentation)
#

img = cv2.imread(img_path)
# img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
pcd = readTXT(pcd_path)

segmentation = np.load("segmentation.npy")
left_u, left_v, length, width = segmentation
img = img[left_v:left_v+width, left_u:left_u+length]
cv2.imshow('img', img)

x = np.arange(left_v, left_v+width)
y = np.arange(left_u, left_u+length)
ind_x, ind_y = np.meshgrid(x, y)
ind_work_space = LENGTH * \
    np.reshape(np.transpose(ind_x), -1) + \
    np.reshape(np.transpose(ind_y), -1)
pcd = select_pcd_by_index(pcd, ind_work_space)
# visualize_pcd([pcd])

pcd = clear_invalide(pcd)
# visualize_pcd([pcd],1)
# pcd_cup = remove_plane(pcd, 10, 0.005)
pcd_cup_ = remove_plane(pcd)
visualize_pcd([pcd_cup_],1)

pcd_cup = pcd_filter_radius(pcd_cup_, 25, 0.03)
# visualize_pcd([pcd_cup], 1)
pcd_cup_down, pcd_cup_fpfh = preprocess_point_cloud(pcd_cup, 0.002)

pcd_model = readPCD(model_path)
# visualize_pcd([pcd])
pcd_model_down, pcd_model_fpfh = preprocess_point_cloud(pcd_model, 0.002)
# visualize_pcd([pcd_down],1)
# visualize_pcd([pcd_cup_down, pcd_model_down], 1)


T, _, _ = execute_global_registration(
    pcd_model_down, pcd_cup_down, pcd_model_fpfh, pcd_cup_fpfh, 0.002)
# visualize_pcd([pcd_cup_down, pcd_model_down.transform(T)], 1)

T, fitness, rmse = execute_local_registration(
    pcd_model_down, pcd_cup_down, trans_init=T)

visualize_pcd([pcd_cup_down, pcd_model_down.transform(T)], 1)