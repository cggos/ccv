#!/usr/bin/env python
"""
Image brightness calculation tool.
Calculates brightness using both grayscale and HSV methods.
"""

import cv2
import argparse
import logging
from libccv.common.imtools import BrightnessProcessor
from libccv.common.logger import init_logger


def main() -> None:
    """
    Main function to calculate image brightness using two methods.
    """
    init_logger()
    parser = argparse.ArgumentParser(
        description="Calculate image brightness using grayscale and HSV methods"
    )
    parser.add_argument("-i", "--img", required=True, type=str, help="input image path")
    args = parser.parse_args()

    img_path: str = args.img

    # Load image
    img_raw = cv2.imread(img_path)

    # Check if image was loaded successfully
    if img_raw is None:
        logging.error(f"Could not load image from {img_path}")
        return

    # Calculate brightness using the new BrightnessProcessor class
    brightness_processor = BrightnessProcessor()
    grayscale_brightness, hsv_brightness = brightness_processor.calculate_brightness(
        img_raw, method="both"
    )

    print(
        f"Image brightness - Grayscale: {grayscale_brightness:.2f}, HSV: {hsv_brightness:.2f}"
    )


if __name__ == "__main__":
    main()
