#!/usr/bin/env python
"""
Image statistics calculation tool.
Calculates correlation between original and Gaussian blurred images.
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt

from libccv.common.image_processor import ImageProcessor
from libccv.common.statistics_processor import StatisticsProcessor


import logging
from libccv.common.logger import init_logger


def main() -> None:
    """
    Main function to calculate image statistics and display Gaussian blur effect.
    """
    init_logger()
    parser = argparse.ArgumentParser(
        description="Calculate correlation between original and Gaussian blurred images"
    )
    parser.add_argument("-i", "--img", required=True, type=str, help="input image path")
    parser.add_argument(
        "-s", "--sigma", default=4.0, type=float, help="Gaussian filter sigma value"
    )
    args = parser.parse_args()

    img_path: str = args.img
    sigma: float = args.sigma

    # Load image
    img = ImageProcessor.load_image(img_path, mode="gray")
    if img is None:
        logging.error(f"Could not load image from {img_path}")
        return

    # Apply Gaussian filter
    im_blur = StatisticsProcessor.apply_gaussian_filter(img, sigma)

    # Calculate correlation coefficient
    correlation = StatisticsProcessor.calculate_correlation(img, im_blur)

    print(
        f"Correlation coefficient between original and blurred image: {correlation:.4f}"
    )

    # Display images
    plt.figure(figsize=(10, 5))
    plt.subplot(121)
    plt.imshow(img, plt.cm.gray)
    plt.title("Original grayscale")
    plt.xticks([]), plt.yticks([])

    plt.subplot(122)
    plt.imshow(im_blur, plt.cm.gray)
    plt.title(f"Gaussian blur (σ={sigma})")
    plt.xticks([]), plt.yticks([])

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
