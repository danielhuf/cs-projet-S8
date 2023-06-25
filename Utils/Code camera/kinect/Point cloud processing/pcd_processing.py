import open3d as o3d
import numpy as np
import cv2
from frame_change import *
from image_processing import *

pcd_path = './models/cube.pcd'
txt_path = './Project1/test_pcd.txt'
color_path = './Project1/colorImg.jpg'
rgb_path = './Project1/registeredRgbImg.jpg'


# Define models.
models = {'red': ['triangularPrism'], 'yellow': [
    'bridge', 'cylinder'], 'green': ['cuboid'], 'blue': ['cube', 'semicylinder']}


def readPCD(path):
    """read poind cloud data from .pcd file, return a open3d.geometry.PointCloud object. 
    """
    pcd = o3d.io.read_point_cloud(path)
    # o3d.visualization.draw([pcd])
    return pcd


def readTXT(path):
    """read point cloud data from .txt file, return a open3d.geometry.PointCloud object. 
    """
    pcd = o3d.io.read_point_cloud(path, format="xyzrgb")
    # o3d.visualization.draw([pcd])
    return pcd


def compose_pcd(points, colors):
    """Combinde points and colors numpy array into pcd object
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def decompose_pcd(pcd):
    """ decompose point cloud into two numpy arrays. The first is the x,y,z position,
    the second is the color information
    """
    return np.asarray(pcd.points), np.asarray(pcd.colors)


def clear_invalide(pcd):
    """Remove zero, nan and infinite points
    pcd: open3d.geometry.PointCloud
    """
    # clear invalid points
    pcd.remove_non_finite_points()
    points = np.asarray(pcd.points)
    ind = np.where(points[:, 2] > 0)
    return pcd.select_by_index(ind[0])


def select_pcd_by_index(pcd, index):
    """ Extract a sub set of point cloud.
    pcd: open3d.geometry.PointCloud
    index: list or numpy array
    """
    return pcd.select_by_index(index)


def visualize_pcd(pcd_set, flag=0):
    """
    Parameters:
        pcd_set: [pcd1,pcd2,...]
        flag: 0-draw 1-draw_geometries and show normals
    """
    # draw the point cloud
    if flag == 0:
        o3d.visualization.draw(pcd_set)
    elif flag == 1:
        o3d.visualization.draw_geometries(pcd_set)
    else:
        o3d.visualization.draw_geometries(pcd_set, point_show_normal=True)


def remove_plane(pcd, threshold=2000, epsilon=0.003,flag=0):
    """Using RANSAC algorithm, remove a plane in the point cloud
    Parameters:
        pcd: 
        threshold: number of inner points
        epsilon: distantce(m) from the plane, within which a point is considered as inner point
    """
    # method provided by open3d library
    if flag==0:    
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.004, ransac_n=3, num_iterations=100)
        pcd.select_by_index(inliers)
        return pcd
    
    # method written by ourselves, its performance is not so good as previous, but can provide a view inside 
    # the algorithm
    else:
        # initialize
        pcd = clear_invalide(pcd)
        pcd_points = np.asarray(pcd.points)
        Nmax = 0
        itr = 0
        num = pcd_points.shape[0]
        points = np.hstack((pcd_points, np.ones((num, 1))))
        index = []
        while Nmax < threshold and itr < 1000:
            # initialize the parameters of a plane x=[a,b,c,d]
            A0 = np.zeros((3, 3))
            while np.dot(A0[0]-A0[1], A0[0]-A0[2]) < 0.001:  # not collinear
                ind = np.random.randint(0, num-1, size=3)
                A0 = pcd_points[ind]
            x = np.linalg.solve(A0, -np.ones((3, 1)))
            x = np.vstack((x, 1))
            x = x/np.linalg.norm(x[0:3])

            #
            A = np.abs(np.dot(points, x))

            # find the best estimation
            if pcd_points[A[:, 0] < epsilon].shape[0] > Nmax:
                index = A[:, 0] < epsilon
                Nmax = pcd_points[index].shape[0]

            itr += 1

        # extract all inner points using the best estimation
        A = pcd_points[index]

        # Least square estimation of the plane1`1`
        A_TA = np.dot(A.T, A)
        xbar = np.dot(np.dot(np.linalg.inv(A_TA), A.T), -np.ones((Nmax, 1)))
        xbar = np.vstack((xbar, 1))
        xbar = xbar/np.linalg.norm(xbar[0:3])

        ind_outlier_points = np.abs(np.dot(points, xbar))[:, 0] > 1.5*epsilon

        return pcd.select_by_index(ind_outlier_points)


def chessboard_corner_2DT3D(pcd, corners_img, corners_world):
    """ Knowing chessboard corners' coordinates in 2D image and point cloud data, find each corner's 3D position in camera frame
    Parameters:
        pcd: must contain all points data in the image even the invalid ones
        corners_img: the corners data obtained from opencv findChessboardCorners method
        corners_world: the 3D position of corners in world frame, we need this because for some points, we cannot find their 3D position in camera frame, so we need to delete the corresponding points in world frame


    Returns:
        corners_img: 2D points after deleting the irresolvable points
        corners_camera: 3D position in camera frame
        corners_world: 3D position in world frame
    """
    pcd_points = np.asarray(pcd.points)
    corners_camera = np.zeros((len(corners_img), 3))
    width = 640
    for i in range(len(corners_img)):
        u = corners_img[i][0][0]  # abscissa
        v = corners_img[i][0][1]  # ordinate
        u_lower = int(u)
        u_upper = u_lower+1
        v_lower = int(v)
        v_upper = v_lower+1

        # because u,v are not integer, we want use linear interpolation to get its 3d position
        # p1  x1   p2
        #     p        -> find p
        # p3  x2   p4
        p1 = pcd_points[u_lower+v_lower*width]
        p2 = pcd_points[u_upper+v_lower*width]
        p3 = pcd_points[u_lower+v_upper*width]
        p4 = pcd_points[u_upper+v_upper*width]
        if p1[2] == 0 or p2[2] == 0 or p3[2] == 0 or p4[2] == 0:
            corners_img[i][0] = np.zeros(2)
            corners_camera[i] = np.zeros(3)
            corners_world[i] = np.zeros(3)
            continue
        alpha = (u-u_lower)
        beta = (v-v_lower)
        x1 = p1+alpha*(p2-p1)
        x2 = p3+alpha*(p4-p3)
        corners_camera[i] = x1+beta*(x2-x1)

    return corners_img, corners_camera, corners_world


def pcd_filter_statistical(pcd, num_neighbors=20, std_ratio=2):
    """ remove outlier points using statistical filter. Be careful, when calling this method, you should change the name of pcd to 
    get the return value. For example, you should not not write like pcd=pcd_filter_radius(pcd), you should write like 
    pcd_filtered=pcd_filter_radius(pcd).

    Parameters:
        num_neighbors: number of neighbor points
        std_ratio: standard deviation
    """
    pcd.remove_statistical_outlier(num_neighbors, std_ratio)

    return pcd


def pcd_filter_radius(pcd, num_neighbors=20, radius=0.1):
    """remove outlier points using radius filter. Be careful, when calling this method, you should change the name of pcd to 
    get the return value. For example, you should not not write like pcd=pcd_filter_radius(pcd), you should write like 
    pcd_filtered=pcd_filter_radius(pcd).

    Parameters:
        num_neighbors: number of neighbor points
        radius: radius of the neighborhood
    """

    # This is the method provided by open3d
    # pcd.remove_radius_outlier(num_neighbors, radius)

    # This is the method we write, to show the principle of the algorithm.
    points, colors = decompose_pcd(pcd)
    dele = []
    for i, p in enumerate(points):
        temp_points = np.copy(points)
        # calculate the distance
        dist = np.linalg.norm(temp_points-p, axis=1)
        # Check if all n neighbors are in the ball
        for n in range(num_neighbors+1):
            # find the nearest neighbor
            ind = np.argmin(dist)
            # If one neighbor are out of the ball, then the point is outlier
            if dist[ind] > radius:
                dele.append(i)
            else:
                # continue to find next nearest neighbor
                dist = np.delete(dist, ind)

    points = np.delete(points, dele, axis=0)
    colors = np.delete(colors, dele, axis=0)
    pcd = compose_pcd(points, colors)
    return pcd


def preprocess_point_cloud(pcd, voxel_size=0, radius_normal=0):
    """ Downsample the point cloud and calculate the fpfh feature
    """
    # print(":: Downsample with a voxel size %.3f." % voxel_size)

    if voxel_size == 0:
        pcd_down = pcd
    else:
        pcd_down = pcd.voxel_down_sample(voxel_size)

    if radius_normal == 0:
        radius_normal = voxel_size*2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)

    # Estimate normals
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    # Orient to camera
    pcd_down.orient_normals_towards_camera_location(
        camera_location=np.array([0., 0., 0.]))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)

    # Calculate FPFH feature
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def execute_global_registration(source_pcd, target_pcd, source_fpfh,
                                target_fpfh, voxel_size):
    """Registration with RANSAC based on fpfh feature

    """

    distance_threshold = voxel_size * 1.5
    # print(":: RANSAC registration on downsampled point clouds.")
    # print("   Since the downsampling voxel size is %.3f," % voxel_size)
    # print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_pcd, target_pcd, source_fpfh, target_fpfh, distance_threshold, 4, checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(10000, 1))
    # print(result)
    return result.transformation, result.fitness, result.inlier_rmse


def execute_local_registration(source_pcd, target_pcd, threshold=0.5, trans_init=np.identity(4)):
    """Registraion with ICP
    Parameters:
        source_pcd: o3d object, source
        target_pcd: o3d object, model
        threshold: threshold for movable distance of source points
        trans_init: initial transformation for source
    """
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    # print(reg_p2p)
    return reg_p2p.transformation, reg_p2p.fitness, reg_p2p.inlier_rmse


def object_recognize_and_find_position(target_pcd, voxel_size):
    """ Recognize object and find its posture
    Parameters:
        souce_pcd: o3d pcd data, 
        voxel-size: dimension in meter, used in fpfh registration
    """
    colors = (np.asarray(target_pcd.colors)*255).astype('uint8')
    colors_hsv = cv2.cvtColor(
        colors.reshape((1, -1, 3)), cv2.COLOR_RGB2HSV)
    mean_color = np.mean(colors_hsv.reshape(-1, 3), axis=0)

    cat_color = 'red'
    if (mean_color > lower_green).all() and (mean_color < upper_green).all():
        cat_color = 'green'
    elif (mean_color > lower_blue).all() and (mean_color < upper_blue).all():
        cat_color = 'blue'
    elif (mean_color > lower_yellow).all() and (mean_color < upper_yellow).all():
        cat_color = 'yellow'

    best_T = np.identity(4)
    best = [np.inf, '']
    for model in models[cat_color]:
        # for model in models.values():
        target_pcd_t = target_pcd
        source_pcd = readPCD('./models/{}.pcd'.format(model))
        source_down, source_fpfh = preprocess_point_cloud(
            source_pcd, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(
            target_pcd_t, voxel_size)

        # visualize_pcd([source_down, target_down])

        # fitness is the ratio of inlier points in all source points, RMSE is root of covariance
        # i.d. The the square root of the sum of the distances between all matched points
        # divided by the number of all points
        T, _, _ = execute_global_registration(
            source_down, target_down, source_fpfh, target_fpfh, voxel_size)
        T, fitness, rmse = execute_local_registration(
            source_down, target_down, trans_init=T)

        score = 3*(1-fitness)+rmse
        print(model, score)
        if score < best[0]:
            best_T = T
            best[0] = score
            best[1] = model
        target_pcd_t = o3d.geometry.PointCloud()

        ######################## visualize the result ############
        # source_down.transform(T)
        # o3d.visualization.draw([source_down, target_down])

    print("The object recognized is", best[1])

    return best_T, best[1]


if __name__ == "__main__":
    pcd, pcd_points, pcd_colors = readTXT(txt_path)
    # pcd, pcd_points, pcd_colors = readPCD(pcd_path)
    visualize_pcd([pcd])
