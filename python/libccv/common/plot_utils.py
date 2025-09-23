#!/usr/bin/env python
"""
Utility functions for common plotting operations.
"""

import matplotlib.pyplot as plt
import numpy as np
from typing import List, Optional, Tuple, Any
import cv2


def plot_histogram(
    image: np.ndarray, bins: int = 256, title: str = "Histogram"
) -> None:
    """
    Plot the histogram of an image.

    Args:
        image: Input image (grayscale or color)
        bins: Number of bins for the histogram
        title: Plot title
    """
    plt.figure()
    if len(image.shape) == 2:
        plt.hist(image.ravel(), bins, [0, 256])
    else:
        color = ("b", "g", "r")
        for i, col in enumerate(color):
            hist = cv2.calcHist([image], [i], None, [bins], [0, 256])
            plt.plot(hist, color=col)
            plt.xlim([0, bins])

    plt.title(title)
    plt.show()


def plot_image(
    image: np.ndarray, title: str = "Image", cmap: Optional[str] = None
) -> None:
    """
    Plot a single image.

    Args:
        image: Input image
        title: Plot title
        cmap: Colormap for grayscale images
    """
    plt.figure()
    plt.imshow(image, cmap=cmap)
    plt.title(title)
    plt.axis("off")
    plt.show()


def plot_images(
    images: List[np.ndarray],
    titles: List[str],
    nrows: int,
    ncols: int,
    figsize: Tuple[int, int] = (10, 10),
) -> None:
    """
    Plot a grid of images.

    Args:
        images: List of images to plot
        titles: List of titles for each image
        nrows: Number of rows in the grid
        ncols: Number of columns in the grid
        figsize: Figure size
    """
    fig, axes = plt.subplots(nrows, ncols, figsize=figsize)
    for i, ax in enumerate(axes.flat):
        if i < len(images):
            if len(images[i].shape) == 2:
                ax.imshow(images[i], cmap="gray")
            else:
                ax.imshow(cv2.cvtColor(images[i], cv2.COLOR_BGR2RGB))
            ax.set_title(titles[i])
            ax.axis("off")
    plt.tight_layout()
    plt.show()
