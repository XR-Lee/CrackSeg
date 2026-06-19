import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt

def compute_histogram(data, bins, range):
    hist, bin_edges = np.histogram(data, bins=bins, range=range)
    return hist, bin_edges

def save_histogram_as_image(hist, bin_edges, output_path, title):
    plt.figure()
    plt.plot(bin_edges[:-1], hist, color='blue')
    plt.title(title)
    plt.xlabel('Intensity')
    plt.ylabel('Frequency')
    plt.grid(True)
    plt.savefig(output_path)
    plt.close()

def print_point_cloud_channels(pcd):
    print("Available point cloud channels:")
    for key, value in pcd.point.items():
        print(f"Channel: {key}, Shape: {value.shape}")

# def main(pcd_path, image_path, pcd_hist_path, image_hist_path):
#     # Load PCD file
#     pcd = o3d.io.read_point_cloud(pcd_path, format='pcd')
#     # intensities = np.asarray(pcd.points)[:, 3]  # Assuming intensity is in the z-coordinate

#     # intensities = np.asarray(pcd.intensities)[:, 3]

#     # if pcd.has_intensity():
#     #     intensities = np.asarray(pcd.points)[:, 2]  # Assuming intensity is in the z-coordinate
#     # else:
#     #     intensities = np.asarray(pcd.point['intensity'])  # Assuming intensity is stored as a custom attribute

#     intensities = np.asarray(pcd.point['intensity'])
#     # Compute PCD histogram
#     pcd_hist, pcd_bin_edges = compute_histogram(intensities, bins=256, range=(0, 255))
#     save_histogram_as_image(pcd_hist, pcd_bin_edges, pcd_hist_path, 'PCD Intensity Histogram')

#     # Load image
#     image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
#     if image is None:
#         print(f"Failed to load image: {image_path}")
#         return

#     # Compute image histogram
#     image_hist, image_bin_edges = compute_histogram(image.flatten(), bins=256, range=(0, 255))
#     save_histogram_as_image(image_hist, image_bin_edges, image_hist_path, 'Image Intensity Histogram')

#     print(f"Histograms saved: {pcd_hist_path}, {image_hist_path}")

def main(pcd_path, image_path, pcd_hist_path, image_hist_path):
    # Load PCD file
    pcd = o3d.t.io.read_point_cloud(pcd_path, format='pcd')
    print_point_cloud_channels(pcd)
    # Extract intensity data
    if 'intensity' in pcd.point:
        intensities = pcd.point['intensity'].numpy()
    else:
        print(f"No intensity channel found in the point cloud: {pcd_path}")
        return

    # Compute PCD histogram
    pcd_hist, pcd_bin_edges = compute_histogram(intensities, bins=256, range=(0, 255))
    save_histogram_as_image(pcd_hist, pcd_bin_edges, pcd_hist_path, 'PCD Intensity Histogram')

    # Load image
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"Failed to load image: {image_path}")
        return

    # Compute image histogram
    image_hist, image_bin_edges = compute_histogram(image.flatten(), bins=256, range=(0, 255))
    save_histogram_as_image(image_hist, image_bin_edges, image_hist_path, 'Image Intensity Histogram')

    print(f"Histograms saved: {pcd_hist_path}, {image_hist_path}")

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Compare point-cloud intensity and image intensity histograms.")
    parser.add_argument("--pcd_path", required=True)
    parser.add_argument("--image_path", required=True)
    parser.add_argument("--pcd_hist_path", required=True)
    parser.add_argument("--image_hist_path", required=True)
    args = parser.parse_args()

    main(args.pcd_path, args.image_path, args.pcd_hist_path, args.image_hist_path)
