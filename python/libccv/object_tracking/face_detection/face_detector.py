#!/usr/bin/env python
"""
Abstract base class for face detectors.
"""

from abc import ABC, abstractmethod
import numpy as np
import cv2
from typing import List, Tuple, Any


class FaceDetector(ABC):
    """
    Abstract base class for face detection algorithms.
    """

    def __init__(self):
        """
        Initialize the face detector.
        """
        pass

    @abstractmethod
    def detect_faces(self, image: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """
        Detect faces in an image.

        Args:
            image: Input image as numpy array

        Returns:
            List of bounding boxes (x, y, w, h) for detected faces
        """
        pass

    def draw_faces(
        self,
        image: np.ndarray,
        faces: List[Tuple[int, int, int, int]],
        color: Tuple[int, int, int] = (255, 255, 255),
        thickness: int = 3,
    ) -> np.ndarray:
        """
        Draw bounding boxes around detected faces.

        Args:
            image: Input image
            faces: List of face bounding boxes (x, y, w, h)
            color: Box color (B, G, R)
            thickness: Box thickness

        Returns:
            Image with face bounding boxes drawn
        """
        img_copy = image.copy()
        for x, y, w, h in faces:
            cv2.rectangle(img_copy, (x, y), (x + w, y + h), color, thickness)
        return img_copy
