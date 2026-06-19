
import json
import cv2
import numpy as np
import os
import shutil
import open3d as o3d
from loguru import logger as logger
from tqdm import tqdm

config_file = "../data/Daiwu_sample/index.json"
path_to_image_folder = "../data/Daiwu_sample/images/"
path_to_vo_file = "../data/Daiwu_sample/vo_pano.txt"
path_to_undistort_img =  "../data/Daiwu_sample/undistort_images/"
mesh_file = "../data/Daiwu_sample/mesh.ply"
trajectory_file = "../data/Daiwu_sample/vo_pano.txt"
mvs_pose_result = "../data/Daiwu_sample/mvs_pose_result.txt"

def load_calib(filename):
    # Load the json file
    with open(filename, 'r') as f:
        data = json.load(f)

    # Get the image resolution width and height
    width = data['camera_para']['w']
    height = data['camera_para']['h']
    # Get the camera intrinsic data
    fx = data['camera_para']['fx']
    fy = data['camera_para']['fy']
    cx = data['camera_para']['cx']
    cy = data['camera_para']['cy']
    k1 = data['camera_para']['k1']
    k2 = data['camera_para']['k2']
    k3 = data['camera_para']['k3']
    k4 = data['camera_para']['k4']

    t_LC = data["Tlc"]

    return width, height, fx, fy, cx, cy, k1, k2, k3, k4, t_LC



# Function to correct distortion
def undistort_image(img, fx, fy, cx, cy, k1, k2, p3, p4):
    h, w = img.shape[:2]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    D = np.array([k1, k2, p3, p4]) # distortion coefficients
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (w, h), cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

# Function to process images
def process_images(image_folder, vo_pano_file, camera_intrinsics, output_folder):
    # Camera intrinsics
    fx, fy, cx, cy, k1, k2, k3, k4 = camera_intrinsics

    # Create output folder if not exists
    os.makedirs(output_folder, exist_ok=True)

    # Read VO pano
    with open(vo_pano_file, 'r') as f:
        vo_pano = f.readlines()

    # Extract timestamps
    timestamps = [float(line.split()[0]) for line in vo_pano]

    # For each image in folder
    for image_name in os.listdir(image_folder):
        # Extract timestamp from image name
        timestamp = float(image_name.rsplit('.', 1)[0])
        
        # If timestamp is in vo_pano
        if timestamp in timestamps:
            # Load image
            img = cv2.imread(os.path.join(image_folder, image_name))

            # Undistort image
            undistorted_img = undistort_image(img, fx, fy, cx, cy, k1, k2, k3, k4)

            # Save image
            cv2.imwrite(os.path.join(output_folder, image_name), undistorted_img)    


def calculate_keyframes(mesh_file, trajectory_file, t_LC, mvs_pose_result_file, vertical_fov=110, horizontal_fov=180):
    mesh = o3d.io.read_triangle_mesh(mesh_file)
    vertices = np.asarray(mesh.vertices)
    logger.info("Vertices number in mesh.ply is {}".format(len(vertices)))
    trajectories = np.loadtxt(trajectory_file)

    # Read transformation matric from camera to lidar
    t_LC = np.array(t_LC).reshape(4, 4)
    t_CL = np.linalg.inv(t_LC) 

    result = []
    num_of_keyframe_number = 0

    point_to_keyframes = {i: [] for i in range(len(vertices))}

    for idx, trajectory in tqdm(enumerate(trajectories), total=len(trajectories), desc='Processing keyframes'): # 获取相机姿态
        timestamp = trajectory[0]
        camera_pose = trajectory[1:4]
        qx, qy, qz, qw = trajectory[4:]
        quaternion = [qw, qx, qy, qz]
        # 将四元数转换为旋转矩阵
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(quaternion)

        # 使用hidden point removal计算在该viewpoint位置处，可见的点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(vertices)
        _, pt_map = pcd.hidden_point_removal(camera_pose, radius=1.0)
        visible_points = np.asarray(pcd.points)[pt_map]

        # 使用相机pose以及相机的视场角，计算在该相机的pose处，可见的点云
        camera_z_direction = np.array([0,0,1])
        # camera_y_direction = np.array([0,1,0]) 

        visible_points_in_cam = rotation_matrix @ (t_CL[:3, :3] @ visible_points.transpose() + t_CL[:3, 3].reshape((3,1))) + np.asarray(camera_pose).reshape((3,1))
        visible_points_in_cam = visible_points_in_cam.transpose()
        norm_vector =  np.linalg.norm(visible_points_in_cam)
        cos_angle_z = np.dot(visible_points_in_cam, camera_z_direction) / norm_vector
        theta_z  = np.arccos(cos_angle_z)
        # cos_angle_y = np.dot(visible_points_in_cam, camera_y_direction) / np.linalg.norm(visible_points_in_cam)
        # theta_y  = np.arccos(cos_angle_y)
        zenith_angle = np.arccos(visible_points_in_cam[:,1] / norm_vector)
        # logger.info("For idx {}, theta_z is {}, theta_y is {}".format(idx, np.rad2deg(theta_z), np.rad2deg(theta_y)))
        # mask_fov = np.logical_and(zenith_angle < np.deg2rad(90 + vertical_fov/2), np.deg2rad(90 - vertical_fov/2) < zenith_angle, \
        #     theta_z < np.deg2rad(horizontal_fov / 2),  np.deg2rad(- horizontal_fov / 2) < theta_z )
        mask_zenith = np.logical_and(np.deg2rad(90 - vertical_fov/2) < zenith_angle, zenith_angle < np.deg2rad(90 + vertical_fov/2))
        mask_theta = np.logical_and(np.deg2rad(- horizontal_fov / 2) < theta_z, theta_z < np.deg2rad(horizontal_fov / 2))
        mask_fov = np.logical_and(mask_zenith, mask_theta)


        # 对该view里面可见的点，添加该keyframe为能看到此点的关键帧
        visible_points_in_fov = visible_points[mask_fov]
        for point in visible_points_in_fov:
            point_to_keyframes[np.where((vertices == point).all(axis=1))[0][0]].append(idx)

        # for point in visible_points:
        #     point_to_keyframes[np.where((vertices == point).all(axis=1))[0][0]].append(idx)

    # 计算结果
    logger.info("point_to_keyframes.items lens is {}".format(len(point_to_keyframes.items())))
    for point, keyframes in point_to_keyframes.items():
        num_of_keyframes = len(keyframes)
        if num_of_keyframes > 0:
            # num_of_keyframe_number += 1
           result.append(f"{vertices[point][0]} {vertices[point][1]} {vertices[point][2]} {num_of_keyframes} {' '.join(map(str, keyframes))}")

    # 写入结果到txt文件
    with open(mvs_pose_result_file, 'w') as file:
        file.write(f"{len(vertices)}\n")
        file.write("\n".join(result))

    print(f"Results written to result.txt")



if __name__=="__main__":
    width, height, fx, fy, cx, cy, k1, k2, k3, k4, t_LC = load_calib(config_file)
    cam_intrinsic = [ fx, fy, cx, cy, k1, k2, k3, k4]
    print('Image Resolution - Width: {}, Height: {}'.format(width, height))
    print('Camera Intrinsic Data - fx: {}, fy: {}, cx: {}, cy: {}'.format(fx, fy, cx, cy))
    # process_images(path_to_image_folder, 
    #                 path_to_vo_file,
    #                 cam_intrinsic,
    #                 path_to_undistort_img)
    transform_matrix_LC = t_LC
    calculate_keyframes(mesh_file,trajectory_file,transform_matrix_LC, mvs_pose_result)



