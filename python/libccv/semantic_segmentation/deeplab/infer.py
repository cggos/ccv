#!/usr/bin/env python
"""
@Project : dl_with_pytorch
@File    : infer.py
@Site    :
@Author  : Gavin Gao
@Date    : 12/24/22 11:56 PM
"""

import cv2
import torch
import argparse
import os
from libccv.semantic_segmentation import SegmentationModelLoader, SegmentationProcessor


def main():
    parser = argparse.ArgumentParser(
        description="Semantic segmentation inference with refactored code"
    )
    parser.add_argument(
        "--model_path", type=str, default="3000.torch", help="Path to trained model"
    )
    parser.add_argument(
        "--image_path", type=str, default="test.jpg", help="Test image path"
    )
    parser.add_argument(
        "--height", type=int, default=900, help="Image height for processing"
    )
    parser.add_argument(
        "--width", type=int, default=900, help="Image width for processing"
    )
    parser.add_argument(
        "--num_classes", type=int, default=3, help="Number of classes for the model"
    )

    args = parser.parse_args()

    modelPath = args.model_path
    imagePath = args.image_path
    height = args.height
    width = args.width
    num_classes = args.num_classes

    # Check if files exist
    if not os.path.exists(modelPath):
        raise FileNotFoundError(f"Model file not found: {modelPath}")
    if not os.path.exists(imagePath):
        raise FileNotFoundError(f"Image file not found: {imagePath}")

    # Load custom trained model
    model = SegmentationModelLoader.load_custom(
        "deeplabv3_resnet50", modelPath, num_classes
    )

    # Create segmentation processor
    processor = SegmentationProcessor(model)

    # Load and process image
    Img = cv2.imread(imagePath)  # load test image
    if Img is None:
        raise ValueError(f"Could not load image from {imagePath}")
    height_orgin, widh_orgin, d = Img.shape  # Get image original size

    # Preprocess image
    input_tensor = processor.preprocess(Img, target_size=(height, width))

    # Run inference
    Prd = processor.infer(input_tensor)

    # Postprocess output
    seg_map = processor.postprocess(Prd, (height_orgin, widh_orgin))

    # Visualize result
    import matplotlib.pyplot as plt

    plt.imshow(Img[:, :, ::-1])  # Show image
    plt.show()
    plt.imshow(seg_map)  # display segmentation map
    plt.show()


if __name__ == "__main__":
    main()
