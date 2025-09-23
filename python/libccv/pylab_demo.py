#!/usr/bin/env python
"""
Pylab image processing demo.
Demonstrates various pylab image processing and visualization operations.
"""

import argparse
from typing import List, Tuple
from PIL import Image
import matplotlib.pylab as pl
import numpy as np


import logging
from libccv.common.logger import init_logger


def main() -> None:
    """
    Main function to demonstrate pylab image processing and visualization operations.
    """
    init_logger()
    parser = argparse.ArgumentParser(
        description="Demonstrate pylab image processing and visualization operations"
    )
    parser.add_argument("-i", "--img", required=True, type=str, help="input image path")
    parser.add_argument(
        "-p", "--points", default=3, type=int, help="number of points to click"
    )
    parser.add_argument(
        "--plot-points",
        nargs=8,
        type=int,
        default=[100, 100, 400, 400, 200, 500, 200, 500],
        help="points to plot (x1, x2, x3, x4, y1, y2, y3, y4)",
    )
    parser.add_argument(
        "-b", "--bins", default=128, type=int, help="number of histogram bins"
    )
    args = parser.parse_args()

    img_path: str = args.img
    num_points: int = args.points
    plot_points: List[int] = args.plot_points
    num_bins: int = args.bins

    # Load image
    try:
        img = Image.open(img_path)
    except Exception as e:
        logging.error(f"Could not load image from {img_path}: {e}")
        return

    # Convert to array and display
    im = pl.array(img)
    print(f"Image shape: {im.shape}, dtype: {im.dtype}")

    pl.figure(figsize=(10, 8))
    pl.imshow(im)

    # Plot predefined points
    x_coords = plot_points[:4]
    y_coords = plot_points[4:]
    pl.plot(x_coords, y_coords, "r*")
    pl.title(f'Plotting: "{img_path}"')
    pl.axis("on")

    # Interactive point selection
    if num_points > 0:
        print(f"Please click {num_points} points")
        clicked_points = pl.ginput(num_points)
        print("You clicked: ", clicked_points)

    # Create grayscale contour plot
    pl.figure(figsize=(10, 8))
    im_gray = pl.array(img.convert("L"), "f")
    print(f"Grayscale image shape: {im_gray.shape}, dtype: {im_gray.dtype}")

    pl.gray()
    pl.contour(im_gray, origin="image")
    pl.title("Grayscale Contour Plot")
    pl.colorbar()

    # Create histogram
    pl.figure(figsize=(10, 8))
    pl.hist(im_gray.flatten(), num_bins)
    pl.title(f"Image Histogram ({num_bins} bins)")
    pl.xlabel("Pixel Value")
    pl.ylabel("Frequency")

    pl.show()


if __name__ == "__main__":
    main()
