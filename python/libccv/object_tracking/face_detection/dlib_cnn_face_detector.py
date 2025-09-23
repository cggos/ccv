#!/usr/bin/env python
"""
Face detection using Dlib's CNN-based detector.
"""

import cv2
import dlib
import numpy as np
from typing import List, Tuple
from .face_detector import FaceDetector


class DlibCnnFaceDetector(FaceDetector):
    """
    Face detector using Dlib's CNN-based face detector.
    """

    def __init__(self, model_path: str, upsample_num_times: int = 1):
        """
        Initialize the Dlib CNN face detector.

        Args:
            model_path: Path to the CNN face detection model file
            upsample_num_times: Number of times to upsample the image for detection
        """
        super().__init__()
        self.upsample_num_times = upsample_num_times
        self.detector = dlib.cnn_face_detection_model_v1(model_path)

    def detect_faces(self, image: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """
        Detect faces in an image using Dlib's CNN-based detector.

        Args:
            image: Input image as numpy array (grayscale or color)

        Returns:
            List of bounding boxes (x, y, w, h) for detected faces
        """
        # Validate inputs
        if image is None:
            raise ValueError("Input image cannot be None")

        if not isinstance(image, np.ndarray):
            raise TypeError("Input image must be a numpy array")

        if image.size == 0:
            raise ValueError("Input image cannot be empty")

        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Detect faces
        rects = self.detector(gray, self.upsample_num_times)

        # Convert dlib rectangles to (x, y, w, h) format
        faces = []
        for rect in rects:
            x = rect.rect.left()
            y = rect.rect.top()
            w = rect.rect.right() - x
            h = rect.rect.bottom() - y
            faces.append((x, y, w, h))

        return faces
