#!/usr/bin/env python
"""
Fisheye mask generation tool.
Creates a circular mask for fisheye images and applies it to the input image.
"""

import cv2
import numpy as np
import argparse
import logging
from libccv.common.logger import init_logger


def main() -> None:
    """
    Main function to generate a fisheye mask and apply it to an input image.
    """
    init_logger()
    parser = argparse.ArgumentParser(
        description="Generate a circular mask for fisheye images"
    )
    parser.add_argument("-i", "--img", required=True, type=str, help="input image path")
    parser.add_argument(
        "-o",
        "--output",
        default="/tmp/fisheye_mask.png",
        type=str,
        help="output mask image path",
    )
    parser.add_argument(
        "-r",
        "--radius-offset",
        default=10,
        type=int,
        help="offset to subtract from radius",
    )
    args = parser.parse_args()

    img_path: str = args.img
    output_path: str = args.output
    radius_offset: int = args.radius_offset

    # Load image
    img = cv2.imread(img_path)

    # Check if image was loaded successfully
    if img is None:
        logging.error(f"Could not load image from {img_path}")
        return

    print(f"Image shape: {img.shape}")
    rows, cols = img.shape[:2]

    # Calculate center and radius for the circular mask
    xc = cols // 2
    yc = rows // 2
    radius = rows // 2 - radius_offset

    # Create circular mask
    mask = np.zeros((rows, cols), np.uint8)
    cv2.circle(mask, (xc, yc), radius, 255, -1)

    # Apply mask to image
    result = cv2.bitwise_and(img, img, mask=mask)

    # Save mask and display result
    cv2.imwrite(output_path, mask)
    print(f"Mask saved to {output_path}")

    cv2.imshow("Masked image", result)
    print("Press any key to close the image window...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
