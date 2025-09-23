#!/usr/bin/env python
"""
Keypoint detection using Keypoint R-CNN.
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import torch
import logging

import torchvision
from torchvision import transforms as T

from libccv.common.utils import show_imgs
from libccv.common.image_processor import ImageProcessor
from libccv.common.logger import init_logger


class KeypointDetector:
    """
    A class for performing keypoint detection using Keypoint R-CNN.
    """

    def __init__(self):
        """
        Initialize the KeypointDetector with a pre-trained Keypoint R-CNN model.
        """
        self.model = torchvision.models.detection.keypointrcnn_resnet50_fpn(
            weights=torchvision.models.detection.KeypointRCNN_ResNet50_FPN_Weights.COCO_V1
        ).eval()

        # Define keypoints and limbs for visualization
        self.keypoints = [
            "nose",
            "left_eye",
            "right_eye",
            "left_ear",
            "right_ear",
            "left_shoulder",
            "right_shoulder",
            "left_elbow",
            "right_elbow",
            "left_wrist",
            "right_wrist",
            "left_hip",
            "right_hip",
            "left_knee",
            "right_knee",
            "left_ankle",
            "right_ankle",
        ]

        self.limbs = [
            [self.keypoints.index("right_eye"), self.keypoints.index("nose")],
            [self.keypoints.index("right_eye"), self.keypoints.index("right_ear")],
            [self.keypoints.index("left_eye"), self.keypoints.index("nose")],
            [self.keypoints.index("left_eye"), self.keypoints.index("left_ear")],
            [
                self.keypoints.index("right_shoulder"),
                self.keypoints.index("right_elbow"),
            ],
            [self.keypoints.index("right_elbow"), self.keypoints.index("right_wrist")],
            [self.keypoints.index("left_shoulder"), self.keypoints.index("left_elbow")],
            [self.keypoints.index("left_elbow"), self.keypoints.index("left_wrist")],
            [self.keypoints.index("right_hip"), self.keypoints.index("right_knee")],
            [self.keypoints.index("right_knee"), self.keypoints.index("right_ankle")],
            [self.keypoints.index("left_hip"), self.keypoints.index("left_knee")],
            [self.keypoints.index("left_knee"), self.keypoints.index("left_ankle")],
            [
                self.keypoints.index("right_shoulder"),
                self.keypoints.index("left_shoulder"),
            ],
            [self.keypoints.index("right_hip"), self.keypoints.index("left_hip")],
            [self.keypoints.index("right_shoulder"), self.keypoints.index("right_hip")],
            [self.keypoints.index("left_shoulder"), self.keypoints.index("left_hip")],
        ]

    def predict(self, image: np.ndarray, threshold: float = 0.9) -> dict:
        """
        Perform keypoint detection on an image.

        Args:
            image: Input image as numpy array (RGB format)
            threshold: Confidence threshold for detections

        Returns:
            Dictionary containing keypoints, scores, and bounding boxes
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

        # Convert image to tensor
        transform = T.Compose([T.ToTensor()])
        img_tensor = transform(image)

        # Forward pass through the model
        with torch.no_grad():
            output = self.model(img_tensor.unsqueeze(0))[0]

        return output

    def draw_keypoints_per_person(
        self,
        img: np.ndarray,
        all_keypoints: np.ndarray,
        all_scores: np.ndarray,
        confs: np.ndarray,
        keypoint_threshold: float = 2.0,
        conf_threshold: float = 0.9,
    ) -> np.ndarray:
        """
        Draw keypoints on an image for each detected person.

        Args:
            img: Input image
            all_keypoints: Keypoints for all detected persons
            all_scores: Keypoint scores
            confs: Confidence scores for each person
            keypoint_threshold: Minimum score for a keypoint to be drawn
            conf_threshold: Minimum confidence for a person detection

        Returns:
            Image with keypoints drawn
        """
        # Initialize a set of colors from the rainbow spectrum
        cmap = plt.get_cmap("rainbow")
        # Create a copy of the image
        img_copy = img.copy()
        # Pick a set of N color-ids from the spectrum
        color_id = np.arange(1, 255, 255 // len(all_keypoints)).tolist()[::-1]

        # Iterate for every person detected
        for person_id in range(len(all_keypoints)):
            # Check the confidence score of the detected person
            if confs[person_id] > conf_threshold:
                # Grab the keypoint-locations for the detected person
                keypoints = all_keypoints[person_id, ...]
                # Grab the keypoint-scores for the keypoints
                scores = all_scores[person_id, ...]
                # Iterate for every keypoint-score
                for kp in range(len(scores)):
                    # Check the confidence score of detected keypoint
                    if scores[kp] > keypoint_threshold:
                        # Convert the keypoint float-array to a python-list of integers
                        keypoint = tuple(
                            map(int, keypoints[kp, :2].detach().numpy().tolist())
                        )
                        # Pick the color at the specific color-id
                        color = tuple(np.asarray(cmap(color_id[person_id])[:-1]) * 255)
                        # Draw a circle over the keypoint location
                        cv2.circle(img_copy, keypoint, 3, color, -1)

        return img_copy

    def draw_skeleton_per_person(
        self,
        img: np.ndarray,
        all_keypoints: np.ndarray,
        all_scores: np.ndarray,
        confs: np.ndarray,
        keypoint_threshold: float = 2.0,
        conf_threshold: float = 0.9,
    ) -> np.ndarray:
        """
        Draw skeleton connections on an image for each detected person.

        Args:
            img: Input image
            all_keypoints: Keypoints for all detected persons
            all_scores: Keypoint scores
            confs: Confidence scores for each person
            keypoint_threshold: Minimum score for a keypoint to be drawn
            conf_threshold: Minimum confidence for a person detection

        Returns:
            Image with skeleton drawn
        """
        # Initialize a set of colors from the rainbow spectrum
        cmap = plt.get_cmap("rainbow")
        img_copy = img.copy()

        # Check if the keypoints are detected
        if len(all_keypoints) > 0:
            # Pick a set of N color-ids from the spectrum
            colors = np.arange(1, 255, 255 // len(all_keypoints)).tolist()[::-1]
            # Iterate for every person detected
            for person_id in range(len(all_keypoints)):
                # Check the confidence score of the detected person
                if confs[person_id] > conf_threshold:
                    # Grab the keypoint-locations for the detected person
                    keypoints = all_keypoints[person_id, ...]
                    # Iterate for every limb
                    for limb_id in range(len(self.limbs)):
                        # Pick the start-point of the limb
                        limb_loc1 = (
                            keypoints[self.limbs[limb_id][0], :2]
                            .detach()
                            .numpy()
                            .astype(np.int32)
                        )
                        # Pick the start-point of the limb
                        limb_loc2 = (
                            keypoints[self.limbs[limb_id][1], :2]
                            .detach()
                            .numpy()
                            .astype(np.int32)
                        )
                        # Consider limb-confidence score as the minimum keypoint score among the two keypoint scores
                        limb_score = min(
                            all_scores[person_id, self.limbs[limb_id][0]],
                            all_scores[person_id, self.limbs[limb_id][1]],
                        )
                        # Check if limb-score is greater than threshold
                        if limb_score > keypoint_threshold:
                            # Pick the color at a specific color-id
                            color = tuple(
                                np.asarray(cmap(colors[person_id])[:-1]) * 255
                            )
                            # Draw the line for the limb
                            cv2.line(
                                img_copy, tuple(limb_loc1), tuple(limb_loc2), color, 3
                            )

        return img_copy

    def visualize_keypoints(self, image: np.ndarray, output: dict) -> None:
        """
        Visualize keypoints and skeleton on an image.

        Args:
            image: Input image (RGB format)
            output: Model output containing keypoints and scores
        """
        # Draw keypoints
        keypoints_img = self.draw_keypoints_per_person(
            image,
            output["keypoints"],
            output["keypoints_scores"],
            output["scores"],
            keypoint_threshold=2,
        )

        # Draw skeleton
        skeletal_img = self.draw_skeleton_per_person(
            image,
            output["keypoints"],
            output["keypoints_scores"],
            output["scores"],
            keypoint_threshold=2,
        )

        # Show images
        show_imgs(keypoints_img, skeletal_img)


def main() -> None:
    """
    Main function to run keypoint detection on an image.
    """
    init_logger()
    parser = argparse.ArgumentParser(
        description="Keypoint detection using Keypoint R-CNN"
    )
    parser.add_argument("-i", "--image", required=True, help="Path to the input image")
    parser.add_argument(
        "-t",
        "--threshold",
        type=float,
        default=0.9,
        help="Confidence threshold for detections",
    )
    args = parser.parse_args()

    # Initialize detector
    detector = KeypointDetector()

    # Load image
    img_raw = ImageProcessor.load_image(args.image, mode="rgb")
    if img_raw is None:
        logging.error(f"Could not load image from {args.image}")
        return

    # Perform prediction
    output = detector.predict(img_raw, args.threshold)

    # Visualize results
    detector.visualize_keypoints(img_raw, output)


if __name__ == "__main__":
    main()
