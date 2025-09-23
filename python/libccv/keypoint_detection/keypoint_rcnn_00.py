#!/usr/bin/env python
"""
Keypoint detection example using Keypoint R-CNN.
This script demonstrates how to use the KeypointDetector class.
"""

import cv2
import numpy as np
import argparse
import logging

from libccv.keypoint_detection.keypoint_rcnn_detector import KeypointDetector
from libccv.common.image_processor import ImageProcessor
from libccv.common.logger import init_logger


def main() -> None:
    """
    Main function to run keypoint detection on an image.
    """
    init_logger()
    parser = argparse.ArgumentParser(
        description="Keypoint detection using Keypoint R-CNN"
    )
    parser.add_argument("-i", "--image", required=True, help="Path to the input image")
    parser.add_argument(
        "-t",
        "--threshold",
        type=float,
        default=0.9,
        help="Confidence threshold for detections",
    )
    args = parser.parse_args()

    # Initialize detector
    detector = KeypointDetector()

    # Load image
    img_raw = ImageProcessor.load_image(args.image, mode="rgb")
    if img_raw is None:
        logging.error(f"Could not load image from {args.image}")
        return

    # Perform prediction
    output = detector.predict(img_raw, args.threshold)

    # Print keypoints
    print("Detected keypoints:")
    print(output["keypoints"])

    # Visualize results
    detector.visualize_keypoints(img_raw, output)


if __name__ == "__main__":
    main()
