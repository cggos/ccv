import cv2
import torch
import torchvision.transforms as T
import argparse
from libccv.common import dataset
from libccv.semantic_segmentation import SegmentationModelLoader, VideoSegmentation


def main():
    parser = argparse.ArgumentParser(description="Real-time video segmentation")
    parser.add_argument(
        "--model_name",
        type=str,
        default="deeplabv3_mobilenet_v3_large",
        help="Model name to use for inference",
    )
    parser.add_argument(
        "--camera_id", type=int, default=2, help="Camera ID for video capture"
    )
    parser.add_argument(
        "--visualization_mode",
        type=str,
        default="overlay",
        choices=["overlay", "side_by_side", "mask_only"],
        help="Visualization mode",
    )
    parser.add_argument(
        "--alpha", type=float, default=0.5, help="Transparency for overlay mode"
    )

    args = parser.parse_args()

    # Load model using the new model loader
    model = SegmentationModelLoader.load_pretrained(args.model_name)

    # Create video segmentation processor
    video_seg = VideoSegmentation(model, camera_id=args.camera_id)

    # Run real-time segmentation
    video_seg.run_realtime_segmentation(mode=args.visualization_mode, alpha=args.alpha)


if __name__ == "__main__":
    main()
