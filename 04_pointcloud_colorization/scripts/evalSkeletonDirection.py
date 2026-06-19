import numpy as np
import cv2
import matplotlib.pyplot as plt

def apply_gaussian_filter(image, kernel_size=(5, 5), sigma=1):
    """
    Apply Gaussian filter to an image to blur/smooth it.

    Args:
        image (2D array): Input image.
        kernel_size (tuple): The size of the Gaussian kernel (width, height).
        sigma (float): The standard deviation of the Gaussian kernel.

    Returns:
        2D array: Blurred image.
    """
    # Apply Gaussian blur
    blurred_image = cv2.GaussianBlur(image, kernel_size, sigma)
    return blurred_image

def display_images(original, modified):
    """
    Display two images side by side for comparison.

    Args:
        original (2D array): Original image.
        modified (2D array): Modified image.
    """
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.imshow(original, cmap='gray')
    plt.title('Original Image')
    plt.axis('off')

    plt.subplot(1, 2, 2)
    plt.imshow(modified, cmap='gray')
    plt.title('Image with Gaussian Filter')
    plt.axis('off')

    plt.show()

def main():
    import argparse

    parser = argparse.ArgumentParser(description="Visualize Gaussian smoothing on a skeleton mask.")
    parser.add_argument("--image_path", required=True, help="Path to a skeleton mask image.")
    parser.add_argument("--sigma", type=float, default=5.0)
    args = parser.parse_args()

    mask_image = cv2.imread(args.image_path, cv2.IMREAD_GRAYSCALE)
    if mask_image is None:
        raise FileNotFoundError(f"Image not loaded: {args.image_path}")

    blurred_mask = apply_gaussian_filter(mask_image, kernel_size=(5, 5), sigma=args.sigma)
    display_images(mask_image, blurred_mask)


if __name__ == "__main__":
    main()
