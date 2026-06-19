import open3d as o3d
import cv2 as cv2
import argparse 
import os 
# import loguru as logger
import logging
import numpy as np
# EDT
from scipy.ndimage import distance_transform_edt as distance
from scipy.ndimage import sobel, gaussian_filter

from skimage import segmentation as skimage_seg
import matplotlib
matplotlib.use('Agg')  # 使用 Agg 后端
from matplotlib import pyplot as plt
from plantcv import plantcv as pcv
from tqdm import tqdm
import json 

# Set up logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

class CrackDataFrame():
    """
    Represents a crack data frame.

    Attributes:
        frame_timestamp (int): The timestamp of the frame.
        pcd (PointCloud): The point cloud data.
        crack_mask (ndarray): The crack mask.
        norm_mask (ndarray): The normal mask.
        distance_mask (ndarray): The distance mask.
    """

    def __init__(self, timestamp, pcd_path):
        """
        Initializes a new instance of the CrackDataFrame class.

        Args:
            timestamp (int): The timestamp of the frame.
            pcd_path (str): The path to the point cloud data file.
        """
        self.frame_timestamp = timestamp
        self.pcd = o3d.io.read_point_cloud(pcd_path)
        self.crack_mask = None
        self.raw_img = None
        self.norm_mask = None
        self.distance_mask = None
        self.points_3d_mask = None
        self.distance_transformed = None
        self.skeleton = None
        self.skeleton_2d_pts = [] # dictionary of crack skeleton, [(pleft_1, pske_1, pright_1), (pleft_2, pske_2, pright_2), ...]
        # self.skeleton_2d_pts = [] # list of crack skeleton, [(pleft_1, pske_1, pright_1), (pleft_2, pske_2, pright_2), ...]
        self.skeleton_3d_info = [] # list of crack skeleton, [(pleft_1, pske_1, pright_1), (pleft_2, pske_2, pright_2), ...] 
        
    def add_crack_mask(self, crack_mask_path):
        self.crack_mask = cv2.imread(crack_mask_path, cv2.IMREAD_GRAYSCALE)
        
    def add_raw_img(self, raw_img_path):
        self.raw_img = cv2.imread(raw_img_path, cv2.IMREAD_COLOR)
        logger.info(f"Read raw image: {raw_img_path}")
        
    def add_norm_mask(self, norm_mask):
        self.norm_mask = norm_mask

    def add_dist_mask(self, distance_mask):
        self.distance_mask = distance_mask
        
    def add_pts_3d_mask(self, points_3d_mask):
        self.points_3d_mask = points_3d_mask
        
    def add_distance_transform_EDT(self, distance_transformed):
        self.distance_transformed = distance_transformed
        
    def add_skeleton(self, skeleton):
        self.skeleton = skeleton


class Crack():
    """
    The Crack class is responsible for processing crack data, generating normal masks and distance masks,
    and saving the results. It contains methods for reading data, generating masks, and saving the masks.

    Attributes:
    - timestamps (list): A list of timestamps for each frame.
    - frames (list): A list of CrackDataFrame objects representing each frame.
    - filtered_pcd_folder (str): The path to the folder containing filtered point cloud data.
    - crack_mask_folder (str): The path to the folder containing crack mask images.
    - norm_mask_folder (str): The path to the folder where normal masks will be saved.
    - distance_mask_folder (str): The path to the folder where distance masks will be saved.
    - intrinsic_matrix (numpy.ndarray): The intrinsic matrix for camera calibration.
    - distortion_coefficients (numpy.ndarray): The distortion coefficients for camera calibration.

    Methods:
    - __init__(data_root_dir, intrinsic_matrix, distortion_coefficients): Initializes a Crack object.
    - process(): Processes the Crack object.
    - read_data(): Reads point cloud data and mask images.
    - read_point_cloud(): Reads point cloud data from the filtered_pcd_folder.
    - read_mask_images(): Reads mask images from the crack_mask_folder.
    - generate_norm_masks(): Generates normal masks for each crack mask.
    - generate_distance_masks(): Generates distance masks for each crack mask.
    - save_results(): Saves the generated masks.
    """

    def __init__(self, data_root_dir, intrinsic_matrix, distortion_coefficients):
        """
        Initializes a Crack object.

        Parameters:
        - data_root_dir (str): The root directory of the data.
        - intrinsic_matrix (numpy.ndarray): The intrinsic matrix for camera calibration.
        - distortion_coefficients (numpy.ndarray): The distortion coefficients for camera calibration.
        """
        self.timestamps = []        
        self.frames = []
        self.data_root_dir = data_root_dir
        self.filtered_pcd_folder = os.path.join(data_root_dir, "filtered_pcd")
        self.crack_mask_folder = os.path.join(data_root_dir, "mask_select")
        self.raw_img_folder = os.path.join(data_root_dir, "raw_images")

        self.norm_mask_folder = os.path.join(data_root_dir, "norm_masks")
        self.distance_mask_folder = os.path.join(data_root_dir, "distance_mask")
        
        self.edt_skeleton_folder = os.path.join(data_root_dir, "edt_skeleton")
        if not os.path.exists(self.edt_skeleton_folder):
            os.makedirs(self.edt_skeleton_folder)

        self.intrinsic_matrix = intrinsic_matrix
        self.distortion_coefficients = distortion_coefficients
        
        self.read_data()

    def process(self):
        """
        Processes the Crack object.
        """
        self.preprocess()

        self.generate_norm_masks()

        self.generate_distance_masks()

        self.save_results()
        
        self.manual_select_skeleton_points(self.frames)
        
        self.compute_skeleton_edge_pts()
        
    def preprocess(self):
        """
        Preprocesses crack mask images to compute distance transform and skeleton from mask images.

        This method performs the following steps for each frame:
        1. Thresholds the crack mask image to obtain a binary image.
        2. Computes the distance transform of the binary image.
        3. Adds the distance transform to the frame.
        4. Computes and adds the skeleton of the input image.
        5. Saves the temporary results of the distance transform and skeleton.

        Returns:
            None
        """

        logger.info("Preprocessing crack mask images, to compute distance transform and skeleton from mask images")
        for frame in tqdm(self.frames):
            _, binary_image = cv2.threshold(frame.crack_mask, 0, 255, cv2.THRESH_BINARY)
            distance_transformed = distance(binary_image)
            frame.add_distance_transform_EDT(distance_transformed)

            # compute and add skeleton
            
            # Set global debug behavior to None (default), "print" (to file), 
            # or "plot" (Jupyter Notebooks or X11)
            pcv.params.debug = "None"
            skeleton = pcv.morphology.skeletonize(mask=binary_image)
            frame.add_skeleton(skeleton)

            # save temp results of EDT and skeleton
            edt_path = os.path.join(self.edt_skeleton_folder, f"{frame.frame_timestamp}_edt.png")
            edt_vis_path = os.path.join(self.edt_skeleton_folder, f"{frame.frame_timestamp}_edt_vis.png")

            skeleton_path = os.path.join(self.edt_skeleton_folder, f"{frame.frame_timestamp}_skeleton.png")

            cv2.imwrite(edt_path, frame.distance_transformed)
            cv2.imwrite(skeleton_path, frame.skeleton)
            
            # close interactive mode
            plt.ioff()
            plt.imshow(distance_transformed, cmap='viridis')
            plt.colorbar()
            plt.title('distance_transformed')
            plt.savefig(edt_vis_path)
            plt.close()
            
            logger.info(f"Saved edt result: {edt_path}")
            logger.info(f"Saved edt visualization result: {edt_vis_path}")
            logger.info(f"Saved skeleton result: {skeleton_path}")

    def generate_norm_masks(self):
        """
        Generates normal masks for each crack mask.

        For each frame, computes normals for each point in the point cloud.
        Projects 3D points to 2D using the camera intrinsic matrix.
        Creates a normal mask by iterating all pixels of the mask and setting the mask value to the corresponding normal value.
        """
        for frame in self.frames:
            import copy
            pcd = copy.deepcopy(frame.pcd)

            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=60))

            points = np.asarray(pcd.points)
            normals = np.asarray(pcd.normals)

            points_2d, _ = cv2.projectPoints(points, rvec=np.zeros((3, 1)), tvec=np.zeros((3, 1)), 
                                            cameraMatrix=self.intrinsic_matrix, distCoeffs=self.distortion_coefficients)
            points_2d = points_2d.squeeze().astype(int)
            
            # logger.info(f"frame.crack_mask.shape: {frame.crack_mask.shape}")
            # norm_mask = np.zeros_like(frame.crack_mask, dtype=np.float32)
            norm_mask = np.zeros((frame.crack_mask.shape[0], frame.crack_mask.shape[1], 3), dtype=np.float32)
            mask_height, mask_width = frame.crack_mask.shape
            for point, normal in zip(points_2d, normals):
                x, y = point
                if 0 <= x < mask_width and 0 <= y < mask_height > 0:
                    # logging.info(f"Normal: {normal}")
                    norm_mask[y, x] = normal

            frame.add_norm_mask(norm_mask)

    def generate_distance_masks(self):
        """
        Generates distance masks for each crack mask.

        For each frame, projects 3D points to 2D using the camera intrinsic matrix.
        Creates a distance mask by iterating all pixels of the mask and setting the mask value to the corresponding distance value.
        """
        for frame in self.frames:
            # pcd = frame.pcd
            import copy
            pcd = copy.deepcopy(frame.pcd)

            points = np.asarray(pcd.points)
            points_2d, _ = cv2.projectPoints(points, rvec=np.zeros((3, 1)), tvec=np.zeros((3, 1)), 
                                            cameraMatrix=self.intrinsic_matrix, distCoeffs=self.distortion_coefficients)
            points_2d = points_2d.squeeze().astype(int)

            logger.info(f"For frame {frame.frame_timestamp}, points size is: {points.shape}, points_2d size is: {points_2d.shape}")
            
            distance_mask = np.zeros_like(frame.crack_mask, dtype=np.float32)
            points_3d_mask = np.zeros((frame.crack_mask.shape[0], frame.crack_mask.shape[1], 3), dtype=np.float32)
            
            mask_height, mask_width = frame.crack_mask.shape
            for point, xyz in zip(points_2d, points):
                x, y = point
                distance = np.linalg.norm(xyz)

                # if 0 <= x < mask_width and 0 <= y < mask_height and frame.crack_mask[y, x] > 0:
                if 0 <= x < mask_width and 0 <= y < mask_height:
                    distance_mask[y, x] = distance
                    points_3d_mask[y, x] = (xyz[0], xyz[1], xyz[2])

            frame.add_dist_mask(distance_mask)
            frame.add_pts_3d_mask(points_3d_mask)
            
    def save_results(self):
        """
        Saves the generated masks.

        Creates the norm_mask_folder and distance_mask_folder if they don't exist.
        Saves the normal masks and distance masks for each frame.
        """
        if not os.path.exists(self.norm_mask_folder):
            os.makedirs(self.norm_mask_folder)
        if not os.path.exists(self.distance_mask_folder):
            os.makedirs(self.distance_mask_folder)

        for frame in self.frames:
            norm_mask_path = os.path.join(self.norm_mask_folder, f"{frame.frame_timestamp}_norm.png")
            distance_mask_path = os.path.join(self.distance_mask_folder, f"{frame.frame_timestamp}_distance.png")
            vis_pt3d_on_img_path = os.path.join(self.distance_mask_folder, f"{frame.frame_timestamp}_3d_pts_on_img.png")
            
            cv2.imwrite(norm_mask_path, frame.norm_mask)
            cv2.imwrite(distance_mask_path, frame.distance_mask)
            
            # Create a copy of the raw image to draw on
            img_with_3d_pts = frame.raw_img.copy()

            # Normalize z-coordinates to [0, 255] for color mapping
            z_values = frame.points_3d_mask[..., 2]  # Extract all z-coordinates

            if len(z_values) > 0:
                z_min, z_max = np.min(z_values), np.max(z_values)
            else:
                z_min, z_max = 0, 1  # Handle case with no points
                
            logger.debug(f"z_min: {z_min}, z_max: {z_max}")

            # Iterate over each pixel in the mask
            mask_height, mask_width, _ = frame.points_3d_mask.shape

            for y in range(mask_height):  # height
                for x in range(mask_width):  # width
                    # Get the 3D point at this pixel
                    point_3d = frame.points_3d_mask[y, x]
                    
                    # Check if it's a valid point (i.e., not all zeros)
                    if not np.all(point_3d == 0):
                        x3d, y3d, z3d = point_3d

                        # Normalize the z value to a 0-255 range for coloring
                        z_normalized = int(255 * (z3d - z_min) / (z_max - z_min)) if z_max != z_min else 0
                        color = (z_normalized, 0, 255 - z_normalized)  # RGB color based on z value

                        # Draw the point on the image
                        cv2.circle(img_with_3d_pts, (x, y), 1, color, -1)  # Mark the 2D location with color
            # Save the image with 3D points visualized
            cv2.imwrite(vis_pt3d_on_img_path, img_with_3d_pts)
                
            logger.info(f"Saved normal mask: {norm_mask_path}")
            logger.info(f"Saved distance mask: {distance_mask_path}")
            logger.info(f"Saved 3D points on image: {vis_pt3d_on_img_path}")

    def manual_select_skeleton_points(self, frames, scale_factor=0.5):
        # Iterate over each frame
        # np.argwhere(frame.skeleton > 0)
        for frame in frames:
            # Initialize an empty list to store selected points
            selected_points = []

            # Create a copy of the raw image to display and draw on
            display_img = frame.raw_img.copy()
            
            # Get potential skeleton points where frame.skeleton > 0
            skeleton_points = np.argwhere(frame.skeleton > 0)
            
            # Draw potential skeleton points on the display image as small circles
            for (x, y) in skeleton_points:
                cv2.circle(display_img, (y, x), 1, (0, 0, 255), -1)  # Red dots

            # Define the callback function for mouse click events
            def select_point(event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN:
                    # Add the selected point (x, y) to the list
                    # selected_points.append((x, y))
                    selected_points.append((y, x))

                    # Add neighbors within radius 5
                    radius = 0
                    for dx in range(-radius, radius + 1):
                        for dy in range(-radius, radius + 1):
                            if dx**2 + dy**2 <= radius**2:
                                neighbor_x = x + dx
                                neighbor_y = y + dy
                                # Ensure the neighbor is within image bounds
                                if 0 <= neighbor_x < frame.raw_img.shape[1] and 0 <= neighbor_y < frame.raw_img.shape[0]:
                                    # selected_points.append((neighbor_x, neighbor_y))
                                    selected_points.append((neighbor_y, neighbor_x))

                    # Draw a small circle to visualize the selection on the display image
                    cv2.circle(display_img, (x, y), radius, (0, 255, 0), -1)
                    # Show the updated frame
                    cv2.imshow('Select Points', display_img)

            # Create a named window
            cv2.namedWindow('Select Points')
            # Set the mouse callback function for the window
            cv2.setMouseCallback('Select Points', select_point)

            while True:
                # Show the current frame's raw image
                cv2.imshow('Select Points', display_img)
                # Wait for key press
                key = cv2.waitKey(1) & 0xFF
                # If the user presses 'q' or closes the window, finish selection
                if key == ord('q') or cv2.getWindowProperty('Select Points', cv2.WND_PROP_VISIBLE) < 1:
                    break

            # Store the selected points in the frame object
            logger.info(f"Selected points: {selected_points}")
            frame.manual_selected_skeleton_pts = selected_points

            # If the user closes the window, exit the loop
            if cv2.getWindowProperty('Select Points', cv2.WND_PROP_VISIBLE) < 1:
                break

        # Close all OpenCV windows
        cv2.destroyAllWindows()

        # Return the list of selected points
        # return [frame.manual_selected_skeleton_pts for frame in frames]
        
        
    def compute_skeleton_edge_pts(self, radius=20, sample_rate=0.0001, search_radius=0.02, epsilon=1e-9):
        """Compute skeleton edge points for each frame.

        Args:
            radius (int, optional): Radius(in pixel) of the neighborhood for computing skeleton direction. Defaults to 10.
            sample_rate (float, optional): Sample rate (in meter) for searching 3D edge points. Defaults to 0.001.
            search_radius (int, optional): Search radius (in meter) for searching 3D edge points. Defaults to 0.1.
            epsilon (int, optional): Epsilon value (in meter) for searching 3D edge points. Defaults to 1e-3.
        """
        result_data = []  # Initialize the result list
        for frame in self.frames:
            frame.skeleton_2d_pts = []
            frame.skeleton_direction_2d = []

            # Find skeleton points in the frame
            skeleton_points = np.argwhere(frame.skeleton > 0)
            
            # TODO: add manual selected points, pop up a window to select the skeleton points
            # skeleton_points = self.manual_select_skeleton_points(frame)
            cnt = 0
            # for point in skeleton_points:
            for point in frame.manual_selected_skeleton_pts:
                # if cnt % 30 == 0 and cnt != 0:
                    y, x = point

                    # Compute skeleton direction
                    # direction_radius = 12 # pixels
                    direction_2d = self.compute_skeleton_direction(frame.skeleton, x, y)

                    # Find left and right edge points based on the skeleton direction
                    left_edge_pt2d, right_edge_pt2d = self.find_edges_by_direction(frame.crack_mask, x, y, direction_2d)

                    logger.info(f"This is the {len(frame.skeleton_2d_pts)}th skeleton point.")
                    logger.info(f"Direction: {direction_2d}, left_edge_pt2d: {left_edge_pt2d}, right_edge_pt2d: {right_edge_pt2d}")
                    
                    # Find local plane in the neighborhood
                    result = self.find_local_plane(frame.points_3d_mask, x, y)
                    if result is None:
                        logger.info(f"Not enough points to define a plane at ({x}, {y}).")
                        continue 
                    else:
                        local_plane_3d, local_plane_centroid_3d = result
                        logger.info(f"Find the local plane, Local plane coefficiency: {local_plane_3d}, local_plane_centroid_3d: {local_plane_centroid_3d}")

                    if left_edge_pt2d and right_edge_pt2d:
                        # Search for left edge point's 3D coordinates
                        left_edge_3d_pt = self.search_3d_edge_points(local_plane_3d, local_plane_centroid_3d,left_edge_pt2d, sample_rate, search_radius, epsilon)
                        # Search for right edge point's 3D coordinates
                        right_edge_3d_pt = self.search_3d_edge_points(local_plane_3d, local_plane_centroid_3d, right_edge_pt2d, sample_rate, search_radius, epsilon)

                        width = np.linalg.norm(np.array(left_edge_3d_pt) - np.array(right_edge_3d_pt))
                        
                        logger.info(f"left_edge_3d_pt: {left_edge_3d_pt}, right_edge_3d_pt: {right_edge_3d_pt}, width is: {width*1e3}(mm)")
                        # Append the skeleton edge points to the frame
                        # frame.skeleton_2d_pts.append({
                        #     'skeleton_pt': (x, y),
                        #     'left_edge_pt2d': left_edge_pt2d,
                        #     'right_edge_pt2d': right_edge_pt2d,
                        #     'direction_2d': direction_2d.tolist(),
                        #     'local_plane_3d': local_plane_3d,
                        #     'left_edge_3d_pt': left_edge_3d_pt,
                        #     'right_edge_3d_pt': right_edge_3d_pt
                        # })
                        
                        frame.skeleton_2d_pts.append({
                            'frame_timestamp': float(frame.frame_timestamp),
                            'skeleton_pt': (int(x), int(y)),
                            'left_edge_pt2d': (int(left_edge_pt2d[0]), int(left_edge_pt2d[1])) if left_edge_pt2d is not None else None,
                            'right_edge_pt2d': (int(right_edge_pt2d[0]), int(right_edge_pt2d[1])) if right_edge_pt2d is not None else None,
                            'direction_2d': [float(direction_2d[0]), float(direction_2d[1])],
                            'local_plane_3d': [float(local_plane_3d[0]), float(local_plane_3d[1]), float(local_plane_3d[2]), float(local_plane_3d[3])],
                            'left_edge_3d_pt': [float(coord) for coord in left_edge_3d_pt] if left_edge_3d_pt is not None else None,
                            'right_edge_3d_pt': [float(coord) for coord in right_edge_3d_pt] if right_edge_3d_pt is not None else None,
                            'width_m': float(width),
                            'width_mm': float(width * 1e3)
                        })

                # cnt += 1
                                
            vis_path = os.path.join(self.edt_skeleton_folder, f"{frame.frame_timestamp}_skeleton_edge_pts.png")
            self.visualize_skeleton_edge_pts(frame, vis_path)
            result_data.extend(frame.skeleton_2d_pts)

        json_file = os.path.join(self.data_root_dir, 'crack_width_3d_results.json')
        with open(json_file, 'w') as jsonfile:
            json.dump(result_data, jsonfile, indent=4)
                    
                    
    def visualize_skeleton_edge_pts(self, frame, vis_path):
        """Plot the left edge pt2d and right edge pt2d on the skeleton image, and mark the crack width for each skeleton pt,  with the arrow points to the skeleton 2d pt. 

        Args:
            frame (_type_): _description_
            vis_path (_type_): _description_
        """
        # 将骨架图像转换为彩色图像，以便我们可以在其上绘制彩色标记
        # skeleton_image = cv2.cvtColor(frame.raw_img.astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)

        skeleton_image = frame.raw_img.copy()
        
        for data in frame.skeleton_2d_pts:
            x, y = data['skeleton_pt']
            left_edge_pt2d = data['left_edge_pt2d']
            right_edge_pt2d = data['right_edge_pt2d']
            left_edge_3d_pt = data['left_edge_3d_pt']
            right_edge_3d_pt = data['right_edge_3d_pt']
            direction = data['direction_2d']    

            # 绘制骨架点
            cv2.circle(skeleton_image, (x, y), 2, (0, 0, 255), -1)  # 红色

            # 绘制左边缘点
            if left_edge_pt2d is not None:
                cv2.circle(skeleton_image, left_edge_pt2d, 2, (255, 0, 0), -1)  # 蓝色

            # 绘制右边缘点
            if right_edge_pt2d is not None:
                cv2.circle(skeleton_image, right_edge_pt2d, 2, (0, 255, 0), -1)  # 绿色

            # 绘制从骨架点到左右边缘点的箭头
            if left_edge_pt2d is not None:
                cv2.arrowedLine(skeleton_image, (x, y), left_edge_pt2d, (255, 0, 0), 1)
            if right_edge_pt2d is not None:
                cv2.arrowedLine(skeleton_image, (x, y), right_edge_pt2d, (0, 255, 0), 1)
                
            # 绘制骨架方向
            cv2.arrowedLine(skeleton_image, (x, y), (x + int(direction[0] * 10), y + int(direction[1] * 10)), (0, 255, 255), 1) # 黄色

            # 计算裂缝宽度
            if left_edge_3d_pt is not None and right_edge_3d_pt is not None:
                width = np.linalg.norm(np.array(left_edge_3d_pt) - np.array(right_edge_3d_pt)) * 1e3  # 转换为毫米
                # 在骨架点附近标记裂缝宽度
                cv2.putText(skeleton_image, f'{width:.2f}mm', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # 保存可视化结果

        cv2.imwrite(vis_path, skeleton_image)        
    
    def sample_3d_points_on_plane(self, plane_coefficients, center_3d_point, sample_rate=0.0001, search_radius=0.03):
        """Sample 3D points on a plane.

        Args:
            plane_coefficients (tuple): Coefficients of the plane equation (a, b, c, d).
            center_3d_point (tuple): Center point of the plane (x0, y0, z0).
            sample_rate (float): Rate at which to sample the points.
            search_radius (float): Radius around the center point to sample points.

        Returns:
            numpy.ndarray: Array of sampled 3D points on the plane.
        """
        sample_rate = 0.0001 # 0.00001m = 0.05mm
        search_radius = 0.03
        
        a, b, c, d = plane_coefficients
        x0, y0, z0 = center_3d_point

        # Calculate the range of sampling points
        x_min, x_max = x0 - search_radius, x0 + search_radius
        y_min, y_max = y0 - search_radius, y0 + search_radius

        # Generate the grid points
        x_range = np.arange(x_min, x_max, sample_rate)
        y_range = np.arange(y_min, y_max, sample_rate)
        x_grid, y_grid = np.meshgrid(x_range, y_range)
        z_grid = (-a * x_grid - b * y_grid - d) / c

        # Convert the grid points to a list of points
        sampled_points = np.vstack((x_grid.ravel(), y_grid.ravel(), z_grid.ravel())).T

        return sampled_points

    def search_3d_edge_points(self, plane_coefficients, local_plane_centroid_3d,edge_2d_pt, sample_rate=0.0001, search_radius=0.05, epsilon=0.00001):
        """Searches for 3D edge points within a given plane.

        Args:
            plane_coefficients (list): The coefficients of the plane equation.
            edge_2d_pt (list): The 2D coordinates of the edge point.
            sample_rate (float): The rate at which to sample 3D points on the plane, Unit(m).
            search_radius (float): The radius within which to search for 3D points.
            epsilon (float): The maximum error allowed for a 3D point to be considered an edge point.

        Returns:
            list: The best 3D point found within the search criteria.
        """
        # Sample 3D points on the local plane
        sampled_points = self.sample_3d_points_on_plane(plane_coefficients, local_plane_centroid_3d, sample_rate, search_radius)

        min_error = float('inf')
        best_3d_pt = None

        for point in sampled_points:
            # Project these neighboring points onto the 2D plane
            points_2d, _ = cv2.projectPoints(point.reshape(1, -1), rvec=np.zeros((3, 1)), tvec=np.zeros((3, 1)), 
                                            cameraMatrix=self.intrinsic_matrix, distCoeffs=self.distortion_coefficients)
            projected_2d = points_2d.squeeze().astype(int)
            
            # Calculate the error with respect to the given 2D edge point
            error = np.linalg.norm(projected_2d - np.array(edge_2d_pt))
            
            if error < min_error:
                min_error = error
                best_3d_pt = point.tolist()
            
            if error < epsilon:
                break
        
        return best_3d_pt
    
    def find_local_plane(self, points_3d_mask, x, y, radius=150):
        """Return local plane of the skeleton point, using the points in the radius.
        Project 2D points to 3D using the projection matrix.

        Args:
            points_3d_mask (np.ndarray): 3D points mask.
            x (int): x-coordinate of the skeleton point.
            y (int): y-coordinate of the skeleton point.
            radius (int): Radius to consider for local plane calculation.

        Returns:
            np.ndarray: Coefficients of the local plane (ax + by + cz + d = 0).
            centroid: 3D centroid of the local points.
        """
        # Extract the points within the radius
        y_min, y_max = max(0, y - radius), min(points_3d_mask.shape[0], y + radius)
        x_min, x_max = max(0, x - radius), min(points_3d_mask.shape[1], x + radius)
        local_points_3d = points_3d_mask[y_min:y_max, x_min:x_max].reshape(-1, 3)

        # Remove zero points (assuming zero points are invalid)
        local_points_3d = local_points_3d[np.any(local_points_3d != 0, axis=1)]

        if local_points_3d.shape[0] < 3:
            return None  # Not enough points to define a plane

        # Fit a plane to the local points using SVD
        centroid = np.mean(local_points_3d, axis=0)
        centered_points = local_points_3d - centroid
        _, _, vh = np.linalg.svd(centered_points)
        normal = vh[2, :]

        # Plane equation: ax + by + cz + d = 0
        d = -np.dot(normal, centroid)
        plane_coefficients = np.append(normal, d)

        return plane_coefficients, centroid

    def compute_3d_crack_width_via_searching(self, radius = 10, epsilon = 1e-3):
        """Compute 3d crack width for each skeleton point
        For each frame:
        1. Extracts skeleton points.
        2. For each skeleton point:
            - Calculates the 2d distance to the nearest edge.
            - Finds the nearest left and right edge points using circular search.
            - Appends the results to skeleton_2d_pts.
        """
                    
        result_data = []  # Initialize the result data list
        # Extract 3d skeleton pts from distance mask
        for frame in self.frames:
            frame.skeleton_3d_info = []
            for skeleton_2d_pt in frame.skeleton_2d_pts:
                # skeleton_3d_pt = skeleton_2d_pt['skeleton_3d_pt']
                # left_edge_pt2d = skeleton_2d_pt['left_edge_pt2d']
                # right_edge_pt2d = skeleton_2d_pt['right_edge_pt2d']
                width = None
                left_edge_3d_pt = skeleton_2d_pt['left_edge_3d_pt']
                right_edge_3d_pt = skeleton_2d_pt['right_edge_3d_pt']
                
                # Ensure left_edge_pt and right_edge_pt are not None
                if left_edge_3d_pt is not None and right_edge_3d_pt is not None:
                    width = np.linalg.norm(np.array(left_edge_3d_pt) - np.array(right_edge_3d_pt))
                    
                    frame.skeleton_3d_info.append({
                        'frame_timestamp': float(frame.frame_timestamp),
                        # 'skeleton_3d_pt': skeleton_3d_pt, 
                        'left_edge_3d_pt': left_edge_3d_pt, 
                        'right_edge_3d_pt': right_edge_3d_pt,
                        'crack_width_3d': float(width),
                        'width_m': float(width),
                        'width_mm': float(width * 1e3)
                    })
                else:
                    frame.skeleton_3d_info.append({
                        'frame_timestamp': float(frame.frame_timestamp),
                        # 'skeleton_3d_pt': frame.points_3d_mask[skeleton_pt[1], skeleton_pt[0]].tolist(),
                        'left_edge_3d_pt': None,
                        'right_edge_3d_pt': None,
                        'crack_width_3d': None,
                        'width_m': None,
                        'width_mm': None
                    })
                    
                if(width != None):
                    logger.info(f"==== The computed Crack width is: {width*1e3}(mm) ====")
            result_data.extend(frame.skeleton_3d_info)

        json_file = os.path.join(self.data_root_dir, 'crack_width_3d_results.json')
        with open(json_file, 'w') as jsonfile:
            json.dump(result_data, jsonfile, indent=4)

    def find_edges(self, mask, distance_transformed, x, y, distance_to_edge):
        search_radius = int(np.ceil(distance_to_edge))
        min_distance_left = float('inf')
        min_distance_right = float('inf')
        left_edge = None
        right_edge = None
                
        for i in range(-search_radius, search_radius + 1):
            for j in range(-search_radius, search_radius + 1):
                dist = np.sqrt(i**2 + j**2)
                if dist <= distance_to_edge:
                    nx, ny = x + j, y + i
                    if 0 <= nx < mask.shape[1] and 0 <= ny < mask.shape[0] and mask[ny, nx] == 0:
                        if nx < x and dist < min_distance_left:
                            min_distance_left = dist
                            left_edge = (nx, ny)
                        elif nx > x and dist < min_distance_right:
                            min_distance_right = dist
                            right_edge = (nx, ny)
        return left_edge, right_edge
        
    def find_edges_by_direction(self, mask, x, y, direction):
        """
        Find the left and right edge points using the direction vector.

        Args:
            mask: 2D array of the crack mask.
            x: x coordinate of the skeleton point.
            y: y coordinate of the skeleton point.
            direction: Direction vector at the skeleton point.

        Returns:
            A tuple of the left edge point and the right edge point.
        """
        normal = np.array([-direction[1], direction[0]])
        left_edge = self.trace_edge(mask, x, y, normal)
        right_edge = self.trace_edge(mask, x, y, -normal)
        return left_edge, right_edge

    def trace_edge(self, mask, x, y, direction):
        """
        Trace along the given direction until an edge point is found.

        Args:
            mask: 2D array of the crack mask.
            x: x coordinate of the starting point.
            y: y coordinate of the starting point.
            direction: Direction vector to trace along.

        Returns:
            The coordinates of the edge point.
        """
        pos = np.array([x, y], dtype=float)
        direction = np.array(direction, dtype=float)
        logger.info(f"pos: {pos}, direction: {direction}, mask.shape: {mask.shape}")
        # '''
        # 循环条件包含三个部分：
        # 0 <= pos[0] < mask.shape[1]：确保当前位置的 x 坐标在掩码的有效范围内。
        # 0 <= pos[1] < mask.shape[0]：确保当前位置的 y 坐标在掩码的有效范围内。
        # mask[int(pos[1]), int(pos[0])] > 0：检查当前掩码位置是否属于裂缝（假设裂缝区域的值大于0）。
        # '''
        while 0 <= pos[0] < mask.shape[1] and 0 <= pos[1] < mask.shape[0] and mask[int(pos[1]), int(pos[0])] > 0:
            # '''
            # 在每次循环中：
            #     将当前位置 pos 沿着 direction 方向移动一次，pos += direction。
            #     例如，如果 direction 是 [1, 0]，则表示每次沿 x 轴正方向移动一步。
            # '''
            pos += direction

        # Move back one step to be slightly inside the edge
        pos -= direction*3

        # Convert position to integer indices and ensure they are within bounds
        pos = np.clip(pos, [0, 0], [mask.shape[1] - 1, mask.shape[0] - 1]).astype(int)
        logger.info(f"Edge pos inside the mask: {pos}")
        
        # return tuple(pos.astype(int))
        return tuple(pos)        
        
    def read_data(self):
        """
        Reads point cloud data and mask images.

        1. read_point_cloud: Read point cloud from filtered_pcd_folder, each point cloud is named as "{timestamp}.pcd".
        Use open3d to read all point cloud and save them inside self.pcd_list .Besides, extract the timestamp for
        each point cloud and save into self.timestamps
        
        2. read_mask_images: Read mask images from crack_mask_folder, based on the self.timestamps list which extracted from read_point_cloud steps. 
        Each mask image is named as "{timestamp}.png". only select the mask which in the timestamps list  
        """
        self.read_point_cloud()
        self.read_mask_images()

    def read_point_cloud(self):
        """
        Reads point cloud data from the filtered_pcd_folder.

        Each point cloud file is named as "{timestamp}.pcd".
        Uses open3d to read all point clouds and saves them inside self.frames.
        Extracts the timestamp for each point cloud and saves it into self.timestamps.
        """
        for filename in os.listdir(self.filtered_pcd_folder):
            if filename.endswith(".pcd"):
                timestamp = float(filename[:-4])
                pcd_path = os.path.join(self.filtered_pcd_folder, filename)
                logger.info(f"Reading point cloud: {pcd_path}")

                frame = CrackDataFrame(timestamp, pcd_path)
                
                self.timestamps.append(timestamp)
                self.frames.append(frame)

    def read_mask_images(self):
        """
        Reads mask images from the crack_mask_folder.

        Based on the self.timestamps list which was extracted from read_point_cloud method,
        reads mask images named as "{timestamp}.png" and adds them to the corresponding frames.
        """
        for frame in self.frames:
            mask_path = os.path.join(self.crack_mask_folder, f"{frame.frame_timestamp}.png")
            raw_img_path = os.path.join(self.raw_img_folder, f"{frame.frame_timestamp}.jpg")
            
            if os.path.exists(mask_path):
                frame.add_crack_mask(mask_path)
                frame.add_raw_img(raw_img_path)
            else:
                logger.info(f"Mask image not found: {mask_path}")
                
    def compute_skeleton_direction(self, skeleton, x, y):
        """
        Compute the direction of the skeleton at point (x, y).

        Args:
            skeleton: 2D array of the skeleton.
            x: x coordinate of the point.
            y: y coordinate of the point.
            radius: Radius of the neighbourhood to consider.

        Returns:
            A tuple representing the direction vector (dx, dy).
        """
        # Extract the neighbourhood
        radius = 4 # pixels
        y_min, y_max = max(0, y - radius), min(skeleton.shape[0], y + radius + 1)
        x_min, x_max = max(0, x - radius), min(skeleton.shape[1], x + radius + 1)
        neighbourhood = skeleton[y_min:y_max, x_min:x_max]

        # Apply Gaussian filter to smooth the image and reduce noise
        smoothed = gaussian_filter(neighbourhood, sigma=3)

        # Compute gradients using Sobel operator
        gx = sobel(smoothed, axis=1)
        gy = sobel(smoothed, axis=0)

        # # Average gradient to get direction
        # direction_x = np.mean(gx)
        # direction_y = np.mean(gy)
        
        # direction = np.array([direction_x, direction_y], dtype=float)
        # norm = np.linalg.norm(direction)
        # if norm != 0:
        #     direction /= norm
        
        from numpy.linalg import eigh

        # Compute the components of the structure tensor
        Jxx = gx**2
        Jxy = gx * gy
        Jyy = gy**2

        # Average the components of the structure tensor
        Jxx = Jxx.mean()
        Jxy = Jxy.mean()
        Jyy = Jyy.mean()

        # Calculate the eigenvalues and eigenvectors of the structure tensor
        tensor = np.array([[Jxx, Jxy], [Jxy, Jyy]])
        eigenvalues, eigenvectors = eigh(tensor)

        # The eigenvector corresponding to the largest eigenvalue gives the principal direction
        principal_eigenvalue_index = np.argmax(eigenvalues)
        direction = eigenvectors[:, principal_eigenvalue_index]

        # Calculate orthogonal direction
        direction = np.array([-direction[1], direction[0]])

        return direction
    
def load_camera_parameters(args):
    if args.camera_config:
        with open(args.camera_config, "r") as f:
            config = json.load(f)

        if "intrinsic_matrix" in config:
            intrinsic_matrix = np.array(config["intrinsic_matrix"], dtype=float)
        else:
            camera = config.get("camera_para", config)
            intrinsic_matrix = np.array(
                [[camera["fx"], 0.0, camera["cx"]],
                 [0.0, camera["fy"], camera["cy"]],
                 [0.0, 0.0, 1.0]],
                dtype=float,
            )

        if "distortion_coefficients" in config:
            distortion_coefficients = np.array(config["distortion_coefficients"], dtype=float)
        else:
            camera = config.get("camera_para", config)
            distortion_coefficients = np.array(
                [
                    camera.get("k1", 0.0),
                    camera.get("k2", 0.0),
                    camera.get("p1", camera.get("k3", 0.0)),
                    camera.get("p2", camera.get("k4", 0.0)),
                    camera.get("k3_radial", 0.0),
                ],
                dtype=float,
            )
        return intrinsic_matrix, distortion_coefficients

    intrinsic_matrix = np.array(
        [[args.fx, 0.0, args.cx],
         [0.0, args.fy, args.cy],
         [0.0, 0.0, 1.0]],
        dtype=float,
    )
    distortion_coefficients = np.array(args.distortion, dtype=float)
    return intrinsic_matrix, distortion_coefficients


def load_selected_points(selected_points_json):
    with open(selected_points_json, "r") as f:
        data = json.load(f)

    selected = {}
    if isinstance(data, dict) and "points" in data:
        for item in data["points"]:
            timestamp = str(item["timestamp"])
            selected[timestamp] = item.get("points_xy", [])
    elif isinstance(data, dict):
        for timestamp, points in data.items():
            selected[str(timestamp)] = points
    else:
        raise ValueError("selected points JSON must be an object or contain a 'points' list")
    return selected


def assign_selected_points(frames, selected_points_json):
    selected = load_selected_points(selected_points_json)
    for frame in frames:
        timestamp = str(frame.frame_timestamp)
        points_xy = selected.get(timestamp, [])
        # Internally manual_selected_skeleton_pts stores (row, col) == (y, x).
        frame.manual_selected_skeleton_pts = [(int(y), int(x)) for x, y in points_xy]
        logger.info(f"Selected {len(frame.manual_selected_skeleton_pts)} points for frame {timestamp}")


def run_precompute(crack):
    crack.preprocess()
    crack.generate_norm_masks()
    crack.generate_distance_masks()
    crack.save_results()


def main():
    parser = argparse.ArgumentParser(description="Compute 3D crack width from filtered PCDs and refined masks.")
    parser.add_argument(
        "--data-root",
        "--data_root_dir",
        dest="data_root_dir",
        required=True,
        help="Dataset root containing filtered_pcd, mask_select, and raw_images.",
    )
    parser.add_argument(
        "--camera-config",
        default=None,
        help="JSON file containing intrinsic_matrix and distortion_coefficients.",
    )
    parser.add_argument(
        "--stage",
        choices=["precompute", "measure", "all"],
        default="all",
        help="precompute writes EDT/skeleton/norm/distance masks; measure/all also computes width.",
    )
    parser.add_argument(
        "--selected-points-json",
        default=None,
        help="Non-interactive skeleton points JSON. Use points_xy in image coordinates.",
    )
    parser.add_argument(
        "--manual-select",
        action="store_true",
        help="Open an OpenCV window to manually select skeleton points before measurement.",
    )
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    parser.add_argument("--fx", type=float, default=4818.200388954926)
    parser.add_argument("--fy", type=float, default=4819.10345841615)
    parser.add_argument("--cx", type=float, default=2032.4178620390019)
    parser.add_argument("--cy", type=float, default=1535.1895959282901)
    parser.add_argument(
        "--distortion",
        type=float,
        nargs=5,
        default=[0.003043514741045163, 0.06634739187544138, -0.000217681797407554, -0.0006654964142658197, 0.0],
        metavar=("K1", "K2", "P1", "P2", "K3"),
        help="OpenCV distortion coefficients.",
    )
    args = parser.parse_args()

    logging.getLogger().setLevel(getattr(logging, args.log_level))
    logger.setLevel(getattr(logging, args.log_level))

    intrinsic_matrix, distortion_coefficients = load_camera_parameters(args)
    crack = Crack(args.data_root_dir, intrinsic_matrix, distortion_coefficients)

    run_precompute(crack)
    if args.stage == "precompute":
        return

    if args.manual_select:
        crack.manual_select_skeleton_points(crack.frames)
    else:
        if not args.selected_points_json:
            raise ValueError("--selected-points-json is required unless --manual-select is enabled")
        assign_selected_points(crack.frames, args.selected_points_json)

    crack.compute_skeleton_edge_pts()
    # crack.compute_3d_crack_width_via_searching()
    
if __name__ == '__main__':
    main()



