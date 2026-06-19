import numpy as np
from scipy.spatial.transform import Rotation as R


# the calibration result T_lidar_camera: [x, y, z, qx, qy, qz, qw] that transforms a 3D point in the camera frame into the LiDAR frame (i.e., p_lidar = T_lidar_camera * p_camera).
t_lidar_camera = [
      0.12287685909972107,
      -0.041141602501821334,
      -0.09488286471992269,
      -0.007490583831922861,
      -0.7052105962637597,
      0.708707522564207,
      0.01885612717589657
    ]

# Given translation and quaternion for T_lidar_camera
# translation = [0.071771636420221, -0.04934294727365431, -0.0677501086411397]
# quaternion = [0.0016643810867518116, 0.7077549833513899, -0.7064335133612649, -0.0056395546749240365]  # Note: [x, y, z, w]
t_lidar_camera_translation = t_lidar_camera[:3]  # Note: [x, y, z]
t_lidar_camera_quaternion = t_lidar_camera[3:]  # Note: [qx, qy, qz, qw]

### t_lidar_camera
# Convert quaternion to rotation matrix
t_lidar_camera_rotation = R.from_quat(t_lidar_camera_quaternion)
t_lidar_camera_rotation_matrix = t_lidar_camera_rotation.as_matrix()

# Create the 4x4 transformation matrix
T = np.zeros((4, 4))
T[:3, :3] = t_lidar_camera_rotation_matrix
T[:3, 3] = t_lidar_camera_translation
T[3, 3] = 1

### t_camera_lidar
# transforms a 3D point in the LiDAR frame into the camera frame
# Compute the inverse transformation matrix
# t_camera_lidar = R.from_quat(t_lidar_camera_quaternion)
t_camera_lidar_matrix = np.linalg.inv(T)
t_camera_lidar_rotation_matrix = t_camera_lidar_matrix[:3, :3]
t_camera_lidar_translation = t_camera_lidar_matrix[:3, 3]

print("\n t_lidar_camera_rotation_matrix is\n", t_lidar_camera_rotation_matrix.tolist())
print("\n t_lidar_camera_translation is\n", t_lidar_camera_translation)

print("\nt_camera_lidar_rotation_matrix is\n", t_camera_lidar_rotation_matrix.tolist())
print("\n\nt_camera_lidar_translation is\n", t_camera_lidar_translation.tolist())
