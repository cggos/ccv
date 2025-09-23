#!/usr/bin/env python
"""
Abstract base class for object detectors.
"""

from abc import ABC, abstractmethod
import numpy as np
from typing import List, Tuple, Any
import cv2


class Detector(ABC):
    """
    Abstract base class for an object detector.
    """

    def __init__(self, model_path: str = None):
        """
        Initialize the detector.

        Args:
            model_path: Path to the model file (optional)
        """
        self.model = self.load_model(model_path)

    @abstractmethod
    def load_model(self, model_path: str = None) -> Any:
        """
        Load the detection model.

        Args:
            model_path: Path to the model file (optional)

        Returns:
            Loaded model
        """
        pass

    @abstractmethod
    def predict(
        self, image: np.ndarray, threshold: float = 0.5
    ) -> Tuple[List[Any], List[str], List[float]]:
        """
        Perform object detection on an image.

        Args:
            image: Input image (numpy array)
            threshold: Detection threshold

        Returns:
            Tuple of (boxes, classes, scores)
        """
        pass

    def draw_predictions(
        self,
        image: np.ndarray,
        boxes: List[Any],
        classes: List[str],
        scores: List[float],
        color: Tuple[int, int, int] = (0, 255, 0),
        thickness: int = 2,
    ) -> np.ndarray:
        """
        Draw predictions on an image.

        Args:
            image: Input image
            boxes: Bounding boxes
            classes: Class names
            scores: Detection scores
            color: Bounding box color (B, G, R)
            thickness: Bounding box thickness

        Returns:
            Image with predictions drawn
        """
        img_copy = image.copy()
        for i in range(len(boxes)):
            pt1 = (int(boxes[i][0][0]), int(boxes[i][0][1]))
            pt2 = (int(boxes[i][1][0]), int(boxes[i][1][1]))
            label = f"{classes[i]}: {scores[i]:.2f}"

            cv2.rectangle(img_copy, pt1, pt2, color, thickness)
            cv2.putText(
                img_copy,
                label,
                (pt1[0], pt1[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                thickness,
            )
        return img_copy
