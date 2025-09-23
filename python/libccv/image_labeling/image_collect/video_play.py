#!/usr/bin/env python

import sys
import cv2
import argparse
import os
import logging
from libccv.common.logger import init_logger


def main():
    init_logger()
    parser = argparse.ArgumentParser(description="Play video file")
    parser.add_argument(
        "--video", default="/tmp/111.mp4", help="Path to the video file"
    )
    args = parser.parse_args()

    # Check if video file exists
    if not os.path.exists(args.video):
        logging.error(f"Video file {args.video} not found.")
        return

    videoCapture = cv2.VideoCapture(args.video)

    # Check if video opened successfully
    if not videoCapture.isOpened():
        logging.error(f"Could not open video file {args.video}")
        return

    fps = (
        videoCapture.get(cv2.CAP_PROP_FPS) or 30.0
    )  # Default to 30 FPS if not available
    size = (
        int(videoCapture.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(videoCapture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    )
    fNUMS = videoCapture.get(cv2.CAP_PROP_FRAME_COUNT)

    print(f"Video info: {size[0]}x{size[1]}, {fps} FPS, {fNUMS} frames")

    try:
        success, frame = videoCapture.read()
        while success:
            cv2.imshow("Video Player", frame)
            # Wait for next frame based on FPS
            if cv2.waitKey(int(1000 / fps)) & 0xFF == ord("q"):
                break
            success, frame = videoCapture.read()  # 获取下一帧
    finally:
        videoCapture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
