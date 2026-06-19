import cv2
import numpy as np
import os
import shutil

def automatic_brightness_and_contrast(image, clip_hist_percent=1):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
    hist_size = len(hist)
    
    accumulator = []
    accumulator.append(float(hist[0]))
    for index in range(1, hist_size):
        accumulator.append(accumulator[index-1] + float(hist[index]))
    
    maximum = accumulator[-1]
    clip_hist_percent *= (maximum / 100.0)
    clip_hist_percent /= 2.0

    min_gray = 0
    while accumulator[min_gray] < clip_hist_percent:
        min_gray += 1

    max_gray = hist_size - 1
    while accumulator[max_gray] >= (maximum - clip_hist_percent):
        max_gray -= 1

    alpha = 255 / (max_gray - min_gray)
    beta = -min_gray * alpha

    auto_result = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    return auto_result

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

def automatic_saturation_and_exposure(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    # Automatically adjust saturation by histogram equalization
    s = cv2.equalizeHist(s)
    
    # Automatically adjust exposure by histogram equalization
    v = cv2.equalizeHist(v)
    
    hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

def process_and_save_images(input_folder, output_folder):
    # 如果输出文件夹存在，则删除并重建
    if os.path.exists(output_folder):
        shutil.rmtree(output_folder)
    os.makedirs(output_folder)
    
    for img_name in os.listdir(input_folder):
        img_path = os.path.join(input_folder, img_name)
        if os.path.isfile(img_path):
            image = cv2.imread(img_path)
            if image is not None:
                image = automatic_brightness_and_contrast(image)
                
                # 应用CLAHE来提高局部对比度
                image = apply_clahe(image, clip_limit=3.0, tile_grid_size=(8, 8))
                
                # Gamma校正来调整整体亮度
                image = adjust_gamma(image, gamma=1.1)
                # image = automatic_saturation_and_exposure(image)
                
                output_path = os.path.join(output_folder, img_name)
                cv2.imwrite(output_path, image)
            else:
                print(f"Error: Could not load image at {img_path}")
        else:
            print(f"Skipped: {img_path} is not a file")


# def main(input_folder):
#     root_path = os.path.dirname(input_folder)
#     output_folder = os.path.join(root_path, "xxx_balanced_color")
#     process_and_save_images(input_folder, output_folder)
#     print(f"Balanced images saved to {output_folder}")

# # Example usage
# input_folder = '/path/to/your/image/folder'
# main(input_folder)

def main(input_folder):
    root_path = os.path.dirname(input_folder)
    output_folder = os.path.join(root_path, "raw_images_balanced_color")
    process_and_save_images(input_folder, output_folder)
    print(f"Balanced images saved to {output_folder}")

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Apply automatic brightness/contrast, CLAHE, and gamma correction.")
    parser.add_argument("--input_folder", required=True, help="Folder containing input images.")
    args = parser.parse_args()

    main(args.input_folder)
