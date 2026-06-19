import numpy as np
from scipy.ndimage import distance_transform_edt as distance
from skimage import segmentation as skimage_seg
import cv2
from matplotlib import pyplot as plt


def compute_sdf1_1(segmentation):
    """
    compute the signed distance map of binary mask
    input: segmentation, shape = (batch_size, class, x, y, z)
    output: the Signed Distance Map (SDM) 
    sdm(x) = 0; x in segmentation boundary
             -inf|x-y|; x in segmentation
             +inf|x-y|; x out of segmentation

    """
    # print(type(segmentation), segmentation.shape)

    segmentation = segmentation.astype(np.uint8)
    if len(segmentation.shape) == 4: # 3D image
        segmentation = np.expand_dims(segmentation, 1)
    normalized_sdf = np.zeros(segmentation.shape)
    if segmentation.shape[1] == 1:
        dis_id = 0
    else:
        dis_id = 1
    for b in range(segmentation.shape[0]): # batch size
        for c in range(dis_id, segmentation.shape[1]): # class_num
            # ignore background
            posmask = segmentation[b][c]
            negmask = ~posmask
            posdis = distance(posmask)
            negdis = distance(negmask)
            boundary = skimage_seg.find_boundaries(posmask, mode='inner').astype(np.uint8)
            sdf = negdis/np.max(negdis) - posdis/np.max(posdis)
            sdf[boundary>0] = 0
            normalized_sdf[b][c] = sdf
    return normalized_sdf


def mask_EDT(mask):
    _, binary_image = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY)
    distance_transformed = distance(binary_image)
    return distance_transformed


def show_mask(mask, ax):
    color = np.array([1.0, 0.1, 0.1, 0.8])
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1)/255* color.reshape(1, 1, -1)
    # print(mask_image.max())
    ax.imshow(mask_image)
    
def show_points(coords, labels, ax, marker_size=175):
    pos_points = coords[labels==1]
    neg_points = coords[labels==0]
    ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)
    ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)   
    
def show_box(box, ax):
    x0, y0 = box[0], box[1]
    w, h = box[2] - box[0], box[3] - box[1]
    ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor='green', facecolor=(0,0,0,0), lw=2))    

def sort_and_downsample(coords, target_size):
    # Sort the coordinates
    sorted_coords = sorted(coords, key=lambda x: (x[0], x[1]))
    
    # Calculate the step size to evenly sample the sorted data
    if len(sorted_coords) > target_size:
        step = len(sorted_coords) // target_size
    else:
        step = 1  # If the list is already smaller than the target size, use all elements

    # Downsample the list using the calculated step
    downsampled_coords = sorted_coords[::step]

    # Ensure the final size does not exceed the target size
    return downsampled_coords[:target_size]

def get_EDT_max(EDT):
    return np.max(EDT)

def EDT_to_pts(EDT,threas=7):
    _, binary_image = cv2.threshold(EDT,threas, 255, cv2.THRESH_BINARY)
    binary_image = binary_image.astype(np.uint8)
    # Find contours
    contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize a list to store key points
    key_points_list = []

    # Sample key points from contours
    i=0
    for contour in contours:
        # Optionally, you can reduce the number of points using approximations or other methods
        for point in contour:
            # Extracting points as tuple (x, y)
            key_points_list.append(tuple(point[0]))

    # print(len(key_points_list))
    # Output the list of key points
    # print(key_points_list)
    key_points_list = sort_and_downsample(key_points_list, 20)
    # print(len(key_points_list))
    # Output the list of key points
    # print(key_points_list)

    input_point = np.array(key_points_list)
    input_label = np.ones((len(key_points_list)))
    return input_point, input_label