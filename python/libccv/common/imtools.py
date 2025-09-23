#!/usr/bin/env python
"""
Image processing tools and utilities.
"""

import os
from sys import intern

import cv2
from PIL import Image
from numpy import dot, uint8, array, histogram, sqrt, linalg
import numpy as np
from typing import List, Tuple, Any, Union


def get_imlist(path, ext=".jpg"):
    """Get list of image files in a directory.

    Args:
        path: Directory path
        ext: File extension to filter (default: ".jpg")

    Returns:
        List of image file paths
    """
    return [os.path.join(path, f) for f in os.listdir(path) if f.endswith(ext)]


def imresize(im, sz):
    """Resize an image using PIL.

    Args:
        im: Input image as numpy array
        sz: Target size as tuple (width, height)

    Returns:
        Resized image as numpy array
    """
    pil_im = Image.fromarray(uint8(im))
    return array(pil_im.resize(sz))


def histeq(im, nbr_bins=256):
    """Histogram equalization of a grayscale image.

    Args:
        im: Input grayscale image
        nbr_bins: Number of bins for histogram (default: 256)

    Returns:
        Tuple of (equalized_image, cumulative_distribution_function)
    """
    # Calculate image histogram
    imhist, bins = histogram(im.flatten(), nbr_bins, normed=True)
    cdf = imhist.cumsum()  # Cumulative distribution function
    cdf = 255 * cdf / cdf[-1]  # Normalize
    # Use linear interpolation of cdf to find new pixel values
    im2 = intern(im.flatten(), bins[:-1], cdf)
    return im2.reshape(im.shape), cdf


def compute_average(imlist):
    """Compute average image from a list of images.

    Args:
        imlist: List of image file paths

    Returns:
        Average image as numpy array
    """
    averageim = array(Image.open(imlist[0]), "f")

    for imname in imlist[1:]:
        try:
            averageim += array(Image.open(imname))
        except Exception as e:
            print(imname + "...skipped, ERROR: " + str(e))
    averageim /= len(imlist)
    return array(averageim, "uint8")


def pca(X):
    """Principal Component Analysis.

    Args:
        X: Training data matrix, with each row as a training sample

    Returns:
        Tuple of (projection_matrix, variance, mean)
    """
    num_data, dim = X.shape

    # Center data
    mean_X = X.mean(axis=0)
    X = X - mean_X

    if dim > num_data:
        # PCA - using compact trick
        M = dot(X, X.T)  # Covariance matrix
        e, EV = linalg.eigh(M)  # Eigenvalues and eigenvectors
        tmp = dot(X.T, EV).T  # Compact trick
        V = tmp[::-1]  # Reverse since last eigenvectors are the ones we want
        S = sqrt(e)[::-1]  # Reverse since eigenvalues are in increasing order
        for i in range(V.shape[1]):
            V[:, i] /= S
    else:
        # PCA - using SVD method
        U, S, V = linalg.svd(X)
        V = V[:num_data]  # Only return first num_data dimensions

    # Return projection matrix, variance and mean
    return V, S, mean_X


def brightness_by_gray(img_raw):
    """Calculate image brightness using grayscale method.

    Args:
        img_raw: Input image as numpy array (BGR format)

    Returns:
        Mean brightness value
    """
    img_gray = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)
    return np.mean(img_gray)


def brightness_by_hsv(img_raw):
    """Calculate image brightness using HSV method.

    Args:
        img_raw: Input image as numpy array (BGR format)

    Returns:
        Mean brightness value from V channel
    """
    img_hsv = cv2.cvtColor(img_raw, cv2.COLOR_BGR2HSV)
    img_v = img_hsv[:, :, 2]
    return np.mean(img_v)


class BrightnessProcessor:
    """A class for processing image brightness calculations."""

    @staticmethod
    def calculate_brightness_grayscale(image: np.ndarray) -> float:
        """Calculate image brightness using grayscale method.

        Args:
            image: Input image as numpy array (BGR format)

        Returns:
            Mean brightness value
        """
        return brightness_by_gray(image)

    @staticmethod
    def calculate_brightness_hsv(image: np.ndarray) -> float:
        """Calculate image brightness using HSV method.

        Args:
            image: Input image as numpy array (BGR format)

        Returns:
            Mean brightness value from V channel
        """
        return brightness_by_hsv(image)

    @classmethod
    def calculate_brightness(
        cls, image: np.ndarray, method: str = "both"
    ) -> Union[float, Tuple[float, float]]:
        """Calculate image brightness using specified method.

        Args:
            image: Input image as numpy array (BGR format)
            method: Method to use ("grayscale", "hsv", or "both")

        Returns:
            Brightness value(s)
        """
        if method == "grayscale":
            return cls.calculate_brightness_grayscale(image)
        elif method == "hsv":
            return cls.calculate_brightness_hsv(image)
        elif method == "both":
            return (
                cls.calculate_brightness_grayscale(image),
                cls.calculate_brightness_hsv(image),
            )
        else:
            raise ValueError("Method must be 'grayscale', 'hsv', or 'both'")
