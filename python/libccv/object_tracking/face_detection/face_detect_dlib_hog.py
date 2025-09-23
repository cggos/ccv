#!/usr/bin/env python
"""
Face detection using Dlib's HOG-based detector - refactored to use the new DlibHogFaceDetector class.
"""

import cv2
import matplotlib.pyplot as plt
import argparse
import logging
from libccv.object_tracking.face_detection.dlib_hog_face_detector import (
    DlibHogFaceDetector,
)
from libccv.common.image_processor import ImageProcessor
from libccv.common.logger import init_logger


def main():
    """
    Main function to detect faces using Dlib's HOG-based detector.
    """
    init_logger()
    parser = argparse.ArgumentParser(
        description="Face detection using Dlib's HOG detector"
    )
    parser.add_argument("-i", "--image", required=True, help="Path to the input image")
    parser.add_argument(
        "-u",
        "--upsample",
        type=int,
        default=1,
        help="Number of times to upsample image for detection",
    )
    args = parser.parse_args()

    # Load the image
    img = ImageProcessor.load_image(args.image)
    if img is None:
        logging.error(f"Could not load image from {args.image}")
        return

    # Convert to grayscale
    gray = ImageProcessor.convert_to_grayscale(img)

    # Show original image
    plt.figure(figsize=(12, 8))
    plt.imshow(gray, cmap="gray")
    plt.title("Original Image")
    plt.show()

    # Initialize detector
    detector = DlibHogFaceDetector(upsample_num_times=args.upsample)

    # Detect faces
    faces = detector.detect_faces(gray)

    # Draw faces on image
    result_img = detector.draw_faces(gray, faces)

    # Show result
    plt.figure(figsize=(12, 8))
    plt.imshow(result_img, cmap="gray")
    plt.title(f"Detected {len(faces)} faces")
    plt.show()


if __name__ == "__main__":
    main()
