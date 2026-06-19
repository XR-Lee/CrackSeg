import numpy as np
import cv2
from sklearn.linear_model import LinearRegression

def improved_box_counting(sub_block, threshold_count=10):
    sub_block = sub_block.astype(np.float64)
    min_val, max_val = np.min(sub_block), np.max(sub_block)
    thresholds = np.linspace(min_val, max_val, threshold_count)
    counts = []
    for t in thresholds:
        binary = sub_block >= t
        count = np.count_nonzero(binary)
        if count > 0:
            counts.append((np.log(1.0 / (t + 1e-5)), np.log(count)))
    if len(counts) < 2:
        return 0
    x, y = zip(*counts)
    x = np.array(x).reshape(-1, 1)
    y = np.array(y)
    model = LinearRegression().fit(x, y)
    return -model.coef_[0]

def compute_fractal_dimension_matrix(gray_image, block_size=16, threshold_count=10):
    h, w = gray_image.shape
    rows, cols = h // block_size, w // block_size
    fractal_matrix = np.zeros((rows, cols))
    for i in range(rows):
        for j in range(cols):
            block = gray_image[i*block_size:(i+1)*block_size, j*block_size:(j+1)*block_size]
            D = improved_box_counting(block, threshold_count)
            fractal_matrix[i, j] = D
    return fractal_matrix

def binarize_fractal_matrix(matrix):
    matrix_scaled = cv2.normalize(matrix, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    _, binary = cv2.threshold(matrix_scaled, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return binary
