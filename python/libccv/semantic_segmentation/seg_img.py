#!/usr/bin/env python
"""
@Project : dl_with_pytorch
@File    : seg_img.py
@Site    : ref: https://learnopencv.com/pytorch-for-beginners-semantic-segmentation-using-torchvision/
@Author  : Gavin Gao
@Date    : 12/24/22 4:30 PM
"""

import argparse
import os
from PIL import Image
from libccv.common import utils
from libccv.semantic_segmentation import SegmentationModelLoader, SegmentationProcessor


def main():
    parser = argparse.ArgumentParser(description="Semantic segmentation for images")
    parser.add_argument(
        "-i",
        "--image",
        type=str,
        required=True,
        help="Path to input image",
    )
    parser.add_argument(
        "--model_name",
        type=str,
        default="fcn_resnet101",
        help="Model name to use for inference",
    )

    args = parser.parse_args()

    image_path = args.image
    model_name = args.model_name

    # Check if image file exists
    if not os.path.exists(image_path):
        raise FileNotFoundError(f"Image file not found: {image_path}")

    # Load image
    img = Image.open(image_path)

    # Load model using the new model loader
    model = SegmentationModelLoader.load_pretrained(model_name)

    # Create segmentation processor
    processor = SegmentationProcessor(model)

    # Preprocess image
    input_tensor = processor.preprocess(img)

    # Run inference
    output = processor.infer(input_tensor)

    # Postprocess output
    seg_map = processor.postprocess(output, (img.height, img.width))

    # Visualize result
    seg_rgb = (
        processor.model.module.decode_segmap(seg_map)
        if hasattr(processor.model, "module")
        else (
            processor.model.decode_segmap(seg_map)
            if hasattr(processor.model, "decode_segmap")
            else utils.dataset.PASCAL_VOC.decode_segmap(seg_map)
        )
    )

    # Show results
    utils.show_imgs(img, seg_rgb)


if __name__ == "__main__":
    main()
