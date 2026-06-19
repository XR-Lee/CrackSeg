import cv2
import numpy as np
import os
import shutil
import math

# DEFAULT_KERNEL_SIZE = 0.03
# DEFAULT_V = 0.6
# RESTRAIN_HIGHLIGHT = 3
# DARK_EDGE = -0.4

DEFAULT_KERNEL_SIZE = 0.06
DEFAULT_V = 0.6
RESTRAIN_HIGHLIGHT = 6
DARK_EDGE = -0.8

def apply_clahe(image, clip_limit=2.0, tile_grid_size=(8, 8)):
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
    l = clahe.apply(l)
    lab = cv2.merge((l, a, b))
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

def adjust_gamma(image, gamma=1.0):
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)

def parallel_mix(v, v_i, src, mean_v):
    for i in range(src.shape[0]):
        for j in range(src.shape[1]):
            delta = v_i[i, j] / 255.0 - mean_v
            r = delta / mean_v
            if delta > 0:
                r *= math.pow((1 + delta), RESTRAIN_HIGHLIGHT)
            gamma = math.pow(2, r)
            if delta > DARK_EDGE:
                x = math.pow(v[i, j] / 255.0, gamma) * 255
                v[i, j] = x
                src[i, j] = [0, 0, 0]
            else:
                dd = -0.4 - r
                bx = math.pow(src[i, j, 0] / 255.0, gamma + dd / 5) * 255
                gx = math.pow(src[i, j, 1] / 255.0, gamma + dd / 5) * 255
                rx = math.pow(src[i, j, 2] / 255.0, gamma + dd / 5) * 255
                src[i, j, 0] = bx * dd
                src[i, j, 1] = gx * dd
                src[i, j, 2] = rx * dd
                v[i, j] = math.pow(v[i, j] / 255.0, gamma) * (1 - dd) * 255

def parallel_bgr(v_i, src, mean_v, mean_d):
    for i in range(src.shape[0]):
        for j in range(src.shape[1]):
            delta = v_i[i, j] / 255.0 - mean_v
            r = delta / mean_v
            gamma = math.pow(2, r + mean_d)
            bx = math.pow(src[i, j, 0] / 255.0, gamma) * 255
            gx = math.pow(src[i, j, 1] / 255.0, gamma) * 255
            rx = math.pow(src[i, j, 2] / 255.0, gamma) * 255
            src[i, j, 0] = bx
            src[i, j, 1] = gx
            src[i, j, 2] = rx

def agc_mean_mix(src, kernel_size=-1, mean_v=DEFAULT_V):
    if src is None:
        return src

    src_hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    hsv_channels = cv2.split(src_hsv)
    v = hsv_channels[2]

    kernel_size = kernel_size if kernel_size != -1 else int(min(src.shape[:2]) * DEFAULT_KERNEL_SIZE)
    kernel_size = kernel_size if kernel_size % 2 else kernel_size - 1
    v_i = cv2.blur(v, (kernel_size, kernel_size))

    mean_c = cv2.mean(v_i)[0] / 255.0
    mean_d = mean_c - mean_v
    if mean_d > DARK_EDGE:
        parallel_mix(v, v_i, src, mean_v)
        cv2.merge(hsv_channels, src_hsv)
        cv2.cvtColor(src_hsv, cv2.COLOR_HSV2BGR, src)
        dst = cv2.add(src, src)
        return dst
    else:
        parallel_bgr(v_i, src, mean_v, mean_d)
        return src

def process_and_save_images(input_folder, output_folder, clip_limit, tile_grid_size, gamma):
    if os.path.exists(output_folder):
        shutil.rmtree(output_folder)
    os.makedirs(output_folder)
    
    for img_name in os.listdir(input_folder):
        img_path = os.path.join(input_folder, img_name)
        if os.path.isfile(img_path):
            image = cv2.imread(img_path)
            if image is not None:
                # Apply CLAHE
                image = apply_clahe(image, clip_limit, tile_grid_size)
                
                # Apply Gamma Correction
                image = adjust_gamma(image, gamma)
                
                # Apply AGC
                # image = agc_mean_mix(image)

                
                output_path = os.path.join(output_folder, img_name)
                cv2.imwrite(output_path, image)
            else:
                print(f"Error: Could not load image at {img_path}")
        else:
            print(f"Skipped: {img_path} is not a file")

def main(input_folder):
    # Set experiment parameters
    clip_limit = 1.0
    tile_grid_size = (8, 8)
    gamma = 0.8

    output_folder = os.path.join(os.path.dirname(input_folder), "image_balanced_color_auto")
    print(f"Processing with clipLimit={clip_limit}, tileGridSize={tile_grid_size}, gamma={gamma}")
    process_and_save_images(input_folder, output_folder, clip_limit, tile_grid_size, gamma)
    print(f"Images saved to {output_folder}")

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Apply autonomous color balancing to an image folder.")
    parser.add_argument("--input_folder", required=True, help="Folder containing input images.")
    args = parser.parse_args()

    main(args.input_folder)

