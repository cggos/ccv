#!/usr/bin/env python
"""
Faster R-CNN detector implementation.
"""

import torch
import torchvision.transforms as T
from torchvision import models
from typing import List, Tuple, Any
import numpy as np

from .detector import Detector
from ..common import dataset


class FasterRCNNDetector(Detector):
    """
    Faster R-CNN object detector implementation.
    """

    def load_model(self, model_path: str = None) -> Any:
        """
        Load the Faster R-CNN model.

        Args:
            model_path: Ignored, loads a pre-trained model from torchvision

        Returns:
            Loaded Faster R-CNN model
        """
        return models.detection.fasterrcnn_resnet50_fpn(
            weights=models.detection.FasterRCNN_ResNet50_FPN_Weights.COCO_V1
        ).eval()

    def predict(
        self, image: np.ndarray, threshold: float = 0.5
    ) -> Tuple[List[Any], List[str], List[float]]:
        """
        Perform object detection on an image using Faster R-CNN.

        Args:
            image: Input image (numpy array)
            threshold: Detection threshold

        Returns:
            Tuple of (boxes, classes, scores)
        """
        # Validate inputs
        if image is None:
            raise ValueError("Input image cannot be None")

        if not isinstance(image, np.ndarray):
            raise TypeError("Input image must be a numpy array")

        if image.size == 0:
            raise ValueError("Input image cannot be empty")

        if not 0.0 <= threshold <= 1.0:
            raise ValueError("Threshold must be between 0.0 and 1.0")

        transform = T.Compose([T.ToTensor()])
        img_tensor = transform(image).unsqueeze(0)

        with torch.no_grad():
            predictions = self.model(img_tensor)

        pred = predictions[0]

        pred_scores = list(pred["scores"].detach().numpy())

        # Filter out predictions below the threshold
        pred_t = [i for i, score in enumerate(pred_scores) if score > threshold]
        if not pred_t:
            return [], [], []

        last_index = pred_t[-1]

        pred_boxes = [
            [(box[0], box[1]), (box[2], box[3])]
            for box in list(pred["boxes"].detach().numpy())
        ][: last_index + 1]

        pred_class_indices = list(pred["labels"].numpy())[: last_index + 1]
        pred_class = [
            dataset.COCO_INSTANCE_CATEGORY_NAMES[i] for i in pred_class_indices
        ]

        pred_scores = pred_scores[: last_index + 1]

        # Apply the threshold again to be sure
        final_boxes = []
        final_classes = []
        final_scores = []
        for i, score in enumerate(pred_scores):
            if score > threshold:
                final_boxes.append(pred_boxes[i])
                final_classes.append(pred_class[i])
                final_scores.append(score)

        return final_boxes, final_classes, final_scores
