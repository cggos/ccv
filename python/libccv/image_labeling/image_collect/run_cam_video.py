#!/usr/bin/env python

import sys
import cv2
import argparse
import os
import logging
from libccv.common.logger import init_logger


def main():
    init_logger()
    parser = argparse.ArgumentParser(description="Record video from camera")
    parser.add_argument("video_dir", help="Directory to save video")
    parser.add_argument("video_name", help="Name of the video file (without extension)")
    parser.add_argument(
        "--device", type=int, default=4, help="Camera device ID (default: 4)"
    )
    parser.add_argument(
        "--width", type=int, default=1280, help="Capture width (default: 1280)"
    )
    parser.add_argument(
        "--height", type=int, default=720, help="Capture height (default: 720)"
    )
    args = parser.parse_args()

    # Ensure output directory exists
    os.makedirs(args.video_dir, exist_ok=True)

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    # Check if camera opened successfully
    if not cap.isOpened():
        logging.error(f"Could not open camera device {args.device}")
        return

    # Get actual frame dimensions
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0  # Default to 30 FPS if not available

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_path = os.path.join(args.video_dir, f"{args.video_name}.mp4")
    out = cv2.VideoWriter(video_path, fourcc, fps, (width, height))

    flag = False
    print("Press 's' to start recording, 'q' or 'ESC' to stop and quit")

    try:
        while True:
            ret, frame = cap.read()
            if ret:
                k = cv2.waitKey(1) & 0xFF
                cv2.imshow("frame", frame)
                if flag:
                    out.write(frame)
                if k == ord("s"):
                    flag = True
                    print(f"start recording video {video_path}")
                elif k == ord("q") or k == 27:  # 'q' or ESC key
                    print("done")
                    break
            else:
                logging.error("Could not read frame from camera")
                break
    finally:
        cap.release()
        out.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
