#!/usr/bin/env python
"""
Object detection example using Faster R-CNN.
"""

import argparse
import cv2

from .faster_rcnn_detector import FasterRCNNDetector
from ..common.image_processor import ImageProcessor


def main() -> None:
    parser = argparse.ArgumentParser(description="Object detection using Faster R-CNN")
    parser.add_argument("-i", "--image", required=True, help="Path to the input image")
    parser.add_argument(
        "-t", "--threshold", type=float, default=0.8, help="Detection threshold"
    )
    args = parser.parse_args()

    # Initialize detector
    detector = FasterRCNNDetector()

    # Load image
    img_raw = ImageProcessor.load_image(args.image, mode="rgb")
    if img_raw is None:
        return

    # Perform prediction
    boxes, classes, scores = detector.predict(img_raw, args.threshold)

    # Draw predictions
    img_with_predictions = detector.draw_predictions(
        cv2.cvtColor(img_raw, cv2.COLOR_RGB2BGR), boxes, classes, scores
    )

    # Display images
    ImageProcessor.show_images_side_by_side(
        [img_raw, cv2.cvtColor(img_with_predictions, cv2.COLOR_BGR2RGB)],
        ["Original", "Predictions"],
    )


if __name__ == "__main__":
    main()
