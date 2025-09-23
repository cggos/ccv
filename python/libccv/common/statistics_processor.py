#!/usr/bin/env python
"""
Statistics processor for calculating image statistics.
"""

import numpy as np
from scipy import ndimage
from typing import Dict, Any


class StatisticsProcessor:
    """
    A class for calculating various image statistics.
    """

    def __init__(self):
        """Initialize the StatisticsProcessor."""
        pass

    @staticmethod
    def calculate_basic_statistics(image: np.ndarray) -> Dict[str, Any]:
        """
        Calculate basic statistics for an image.

        Args:
            image: Input image

        Returns:
            Dictionary with statistics
        """
        stats = {
            "shape": image.shape,
            "dtype": image.dtype,
            "min": np.min(image),
            "max": np.max(image),
            "mean": np.mean(image),
            "std": np.std(image),
        }

        if len(image.shape) == 3:
            stats["channels"] = image.shape[2]
        else:
            stats["channels"] = 1

        return stats

    @staticmethod
    def calculate_correlation(image1: np.ndarray, image2: np.ndarray) -> float:
        """
        Calculate the correlation coefficient between two images.

        Args:
            image1: First input image
            image2: Second input image

        Returns:
            Correlation coefficient
        """
        if image1.shape != image2.shape:
            raise ValueError("Input images must have the same shape.")

        correlation_matrix = np.corrcoef(image1.flatten(), image2.flatten())
        return correlation_matrix[0, 1]

    @staticmethod
    def apply_gaussian_filter(image: np.ndarray, sigma: float) -> np.ndarray:
        """
        Apply a Gaussian filter to an image.

        Args:
            image: Input image
            sigma: Standard deviation for Gaussian kernel

        Returns:
            Filtered image
        """
        return ndimage.gaussian_filter(image, sigma)
