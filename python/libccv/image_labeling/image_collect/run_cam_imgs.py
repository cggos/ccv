#!/usr/bin/env python

import sys
import cv2
import argparse
import os
import logging
from libccv.common.logger import init_logger


def main():
    init_logger()
    parser = argparse.ArgumentParser(description="Capture images from camera")
    parser.add_argument("img_dir", help="Directory to save images")
    parser.add_argument("idx_s", help="Starting index for image naming")
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
    os.makedirs(args.img_dir, exist_ok=True)

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    # Check if camera opened successfully
    if not cap.isOpened():
        logging.error(f"Could not open camera device {args.device}")
        return

    i = int(args.idx_s)
    print("Press 's' to save image, 'ESC' to quit")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                logging.error("Could not read frame from camera")
                break

            k = cv2.waitKey(1) & 0xFF
            if k == 27:  # ESC key
                break
            elif k == ord("s"):
                img_name = os.path.join(
                    args.img_dir, f"rs_{i:0>5d}.jpg"
                )  # jpg for PASCAL VOC dataset format
                print(f"saved {img_name}")
                cv2.imwrite(img_name, frame)
                i += 1
            cv2.imshow("capture", frame)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
