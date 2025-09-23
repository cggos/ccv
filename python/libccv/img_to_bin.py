#!/usr/bin/env python
"""
Image to binary file converter.
Converts an image to a binary file format.
"""

import argparse
from typing import Optional
import cv2
import logging
from libccv.common.logger import init_logger


def main() -> None:
    """
    Main function to convert an image to binary format.
    """
    init_logger()
    parser = argparse.ArgumentParser(
        description="Convert an image to binary file format"
    )
    parser.add_argument("-i", "--img", required=True, type=str, help="input image path")
    parser.add_argument(
        "-o",
        "--output",
        default="/tmp/out_img.bin",
        type=str,
        help="output binary file path",
    )
    args = parser.parse_args()

    img_path: str = args.img
    output_path: str = args.output

    # Load image
    img = cv2.imread(img_path)

    # Check if image was loaded successfully
    if img is None:
        logging.error(f"Could not load image from {img_path}")
        return

    print(f"Image type: {type(img)}")
    print(f"Image shape: {img.shape}")

    # Save as binary file
    img.tofile(output_path)
    print(f"Binary file saved to {output_path}")


if __name__ == "__main__":
    main()
