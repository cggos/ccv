#!/usr/bin/env python
"""
Face detection using Haar cascades.
"""

import cv2
import numpy as np
from typing import List, Tuple
from .face_detector import FaceDetector


class HaarFaceDetector(FaceDetector):
    """
    Face detector using Haar cascades from OpenCV.
    """

    def __init__(self, scaleFactor: float = 1.1, minNeighbors: int = 5):
        """
        Initialize the Haar face detector.

        Args:
            scaleFactor: Parameter specifying how much the image size is reduced at each image scale
            minNeighbors: Parameter specifying how many neighbors each candidate rectangle should retain
        """
        super().__init__()
        self.scaleFactor = scaleFactor
        self.minNeighbors = minNeighbors
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        )
        self.eye_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + "haarcascade_eye.xml"
        )

    def detect_faces(self, image: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """
        Detect faces in an image using Haar cascades.

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
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scaleFactor,
            minNeighbors=self.minNeighbors,
            flags=cv2.CASCADE_SCALE_IMAGE,
        )

        return [(x, y, w, h) for (x, y, w, h) in faces]

    def detect_eyes(
        self, image: np.ndarray, face_roi: np.ndarray
    ) -> List[Tuple[int, int, int, int]]:
        """
        Detect eyes in a face region.

        Args:
            image: Input image
            face_roi: Region of interest (face region) as (x, y, w, h)

        Returns:
            List of bounding boxes (x, y, w, h) for detected eyes
        """
        x, y, w, h = face_roi
        face_region = image[y : y + h, x : x + w]

        # Convert to grayscale if needed
        if len(face_region.shape) == 3:
            gray_face = cv2.cvtColor(face_region, cv2.COLOR_BGR2GRAY)
        else:
            gray_face = face_region

        # Detect eyes
        eyes = self.eye_cascade.detectMultiScale(gray_face)
        return [(x + ex, y + ey, ew, eh) for (ex, ey, ew, eh) in eyes]
