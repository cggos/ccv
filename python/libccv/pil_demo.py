#!/usr/bin/env python
"""
PIL image processing demo.
Demonstrates various PIL image processing operations.
"""

import argparse
from typing import Tuple
from PIL import Image
import logging
from libccv.common.logger import init_logger


def main() -> None:
    """
    Main function to demonstrate PIL image processing operations.
    """
    init_logger()
    parser = argparse.ArgumentParser(
        description="Demonstrate PIL image processing operations"
    )
    parser.add_argument("-i", "--img", required=True, type=str, help="input image path")
    parser.add_argument("-o", "--output", type=str, help="output image path (optional)")
    parser.add_argument(
        "--crop-box",
        nargs=4,
        type=int,
        default=[50, 50, 400, 400],
        help="crop box coordinates (left, upper, right, lower)",
    )
    parser.add_argument(
        "-r", "--rotate", default=45, type=int, help="rotation angle in degrees"
    )
    parser.add_argument(
        "--size",
        nargs=2,
        type=int,
        default=[128, 128],
        help="output image size (width, height)",
    )
    args = parser.parse_args()

    img_path: str = args.img
    output_path: str = args.output
    crop_box: Tuple[int, int, int, int] = tuple(args.crop_box)  # type: ignore
    rotation_angle: int = args.rotate
    output_size: Tuple[int, int] = tuple(args.size)  # type: ignore

    # Load image
    try:
        pil_im = Image.open(img_path).convert("L")
    except Exception as e:
        logging.error(f"Could not load image from {img_path}: {e}")
        return

    # Crop and manipulate a region
    try:
        region = pil_im.crop(crop_box)
        region = region.transpose(Image.ROTATE_180)
        pil_im.paste(region, crop_box)
    except Exception as e:
        logging.warning(f"Could not process crop region: {e}")

    # Rotate the entire image
    pil_im = pil_im.rotate(rotation_angle)

    # Resize the image
    pil_im.thumbnail(output_size)
    pil_im = pil_im.resize(output_size)

    # Save if output path is provided
    if output_path:
        try:
            pil_im.save(output_path)
            print(f"Image saved to {output_path}")
        except Exception as e:
            logging.error(f"Could not save image to {output_path}: {e}")

    # Display the image
    pil_im.show()


if __name__ == "__main__":
    main()
