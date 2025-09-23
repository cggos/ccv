#!/usr/bin/env python
"""
Base ImageProcessor class for common image processing operations.
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List
import matplotlib.pyplot as plt
import logging
from libccv.common.logger import init_logger

init_logger()


class ImageProcessor:
    """
    A base class for common image processing operations.
    """

    def __init__(self):
        """Initialize the ImageProcessor."""
        pass

    @staticmethod
    def load_image(filepath: str, mode: str = "color") -> Optional[np.ndarray]:
        """
        Load an image from file.

        Args:
            filepath: Path to the image file
            mode: "color" for BGR, "gray" for grayscale, "rgb" for RGB

        Returns:
            Loaded image as numpy array, or None if failed
        """
        try:
            if mode == "gray":
                img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
            else:
                img = cv2.imread(filepath)
                if img is not None and mode == "rgb":
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            return img
        except Exception as e:
            logging.error(f"Error loading image {filepath}: {e}")
            return None

    @staticmethod
    def save_image(image: np.ndarray, filepath: str, quality: int = 95) -> bool:
        """
        Save an image to file.

        Args:
            image: Image to save
            filepath: Path to save the image
            quality: JPEG quality (0-100)

        Returns:
            True if successful, False otherwise
        """
        try:
            if filepath.lower().endswith(".jpg") or filepath.lower().endswith(".jpeg"):
                return cv2.imwrite(
                    filepath, image, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
                )
            else:
                return cv2.imwrite(filepath, image)
        except Exception as e:
            logging.error(f"Error saving image {filepath}: {e}")
            return False

    @staticmethod
    def convert_to_grayscale(image: np.ndarray) -> np.ndarray:
        """
        Convert image to grayscale.

        Args:
            image: Input image

        Returns:
            Grayscale image
        """
        if len(image.shape) == 2:
            return image
        elif len(image.shape) == 3 and image.shape[2] == 1:
            return image.squeeze()
        else:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def resize_image(
        image: np.ndarray, size: Tuple[int, int], interpolation: int = cv2.INTER_LINEAR
    ) -> np.ndarray:
        """
        Resize an image.

        Args:
            image: Input image
            size: Target size (width, height)
            interpolation: Interpolation method

        Returns:
            Resized image
        """
        return cv2.resize(image, size, interpolation=interpolation)

    @staticmethod
    def rotate_image(
        image: np.ndarray,
        angle: float,
        center: Optional[Tuple[int, int]] = None,
        scale: float = 1.0,
    ) -> np.ndarray:
        """
        Rotate an image by a given angle.

        Args:
            image: Input image
            angle: Rotation angle in degrees
            center: Rotation center (x, y). If None, uses image center
            scale: Scaling factor

        Returns:
            Rotated image
        """
        rows, cols = image.shape[:2]

        if center is None:
            center = (cols // 2, rows // 2)

        rotation_matrix = cv2.getRotationMatrix2D(center, angle, scale)
        return cv2.warpAffine(image, rotation_matrix, (cols, rows))

    @staticmethod
    def apply_gaussian_blur(
        image: np.ndarray, kernel_size: Tuple[int, int], sigma_x: float = 0
    ) -> np.ndarray:
        """
        Apply Gaussian blur to an image.

        Args:
            image: Input image
            kernel_size: Kernel size (width, height)
            sigma_x: Gaussian kernel standard deviation in X direction

        Returns:
            Blurred image
        """
        return cv2.GaussianBlur(image, kernel_size, sigma_x)

    @staticmethod
    def adjust_brightness_contrast(
        image: np.ndarray, brightness: int = 0, contrast: int = 0
    ) -> np.ndarray:
        """
        Adjust brightness and contrast of an image.

        Args:
            image: Input image
            brightness: Brightness adjustment (-255 to 255)
            contrast: Contrast adjustment (-255 to 255)

        Returns:
            Adjusted image
        """
        # Convert to float to prevent overflow
        img = image.astype(np.float32)

        # Apply brightness and contrast adjustments
        img = img * (contrast / 127 + 1) - contrast + brightness

        # Clip values to [0, 255] range
        img = np.clip(img, 0, 255)

        # Convert back to uint8
        return img.astype(np.uint8)

    @staticmethod
    def crop_image(image: np.ndarray, box: Tuple[int, int, int, int]) -> np.ndarray:
        """
        Crop an image.

        Args:
            image: Input image
            box: Crop box (x, y, width, height)

        Returns:
            Cropped image
        """
        x, y, w, h = box
        return image[y : y + h, x : x + w]

    @staticmethod
    def draw_rectangle(
        image: np.ndarray,
        pt1: Tuple[int, int],
        pt2: Tuple[int, int],
        color: Tuple[int, int, int] = (0, 255, 0),
        thickness: int = 2,
    ) -> np.ndarray:
        """
        Draw a rectangle on an image.

        Args:
            image: Input image
            pt1: First point (x, y)
            pt2: Second point (x, y)
            color: Rectangle color (B, G, R)
            thickness: Line thickness

        Returns:
            Image with rectangle
        """
        img_copy = image.copy()
        cv2.rectangle(img_copy, pt1, pt2, color, thickness)
        return img_copy

    @staticmethod
    def draw_text(
        image: np.ndarray,
        text: str,
        org: Tuple[int, int],
        font_face: int = cv2.FONT_HERSHEY_SIMPLEX,
        font_scale: float = 1.0,
        color: Tuple[int, int, int] = (0, 255, 0),
        thickness: int = 2,
    ) -> np.ndarray:
        """
        Draw text on an image.

        Args:
            image: Input image
            text: Text to draw
            org: Text origin point (x, y)
            font_face: Font type
            font_scale: Font scale factor
            color: Text color (B, G, R)
            thickness: Text thickness

        Returns:
            Image with text
        """
        img_copy = image.copy()
        cv2.putText(img_copy, text, org, font_face, font_scale, color, thickness)
        return img_copy

    @staticmethod
    def show_images_side_by_side(
        images: List[np.ndarray],
        titles: Optional[List[str]] = None,
        figsize: Tuple[int, int] = (12, 6),
    ) -> None:
        """
        Display multiple images side by side.

        Args:
            images: List of images to display
            titles: List of titles for each image
            figsize: Figure size (width, height)
        """
        n = len(images)
        if n == 0:
            return

        if titles is None:
            titles = [f"Image {i+1}" for i in range(n)]

        plt.figure(figsize=figsize)

        for i, (img, title) in enumerate(zip(images, titles)):
            plt.subplot(1, n, i + 1)

            # Handle different image types
            if len(img.shape) == 2:
                plt.imshow(img, cmap="gray")
            else:
                # Assume BGR and convert to RGB for matplotlib
                img_rgb = (
                    cv2.cvtColor(img, cv2.COLOR_BGR2RGB) if img.shape[2] == 3 else img
                )
                plt.imshow(img_rgb)

            plt.title(title)
            plt.axis("off")

        plt.tight_layout()
        plt.show()

    @staticmethod
    def calculate_image_statistics(image: np.ndarray) -> dict:
        """
        Calculate basic statistics for an image.

        Args:
            image: Input image

        Returns:
            Dictionary with statistics
        """
        from .statistics_processor import StatisticsProcessor

        return StatisticsProcessor.calculate_basic_statistics(image)


if __name__ == "__main__":
    # Example usage
    # processor = ImageProcessor()
    # img = processor.load_image('path_to_image.jpg')
    # if img is not None:
    #     stats = processor.calculate_image_statistics(img)
    #     print(stats)
    pass
