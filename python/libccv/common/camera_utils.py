#!/usr/bin/env python
"""
Camera utility class for common camera operations.
"""

import cv2
import numpy as np
from typing import Tuple, Optional
import os
import logging
from typing import Tuple, Optional
from libccv.common.logger import init_logger

init_logger()


class CameraCapture:
    """
    A utility class for capturing images and video from camera devices.
    """

    def __init__(
        self, device_id: int = 0, width: int = 640, height: int = 480, fps: int = 30
    ):
        """
        Initialize the CameraCapture.

        Args:
            device_id: Camera device ID (default: 0)
            width: Capture width (default: 640)
            height: Capture height (default: 480)
            fps: Frames per second (default: 30)
        """
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None

    def start(self) -> bool:
        """
        Start the camera capture.

        Returns:
            True if successful, False otherwise
        """
        try:
            self.cap = cv2.VideoCapture(self.device_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            return self.cap.isOpened()
        except Exception as e:
            logging.error(f"Error starting camera: {e}")
            return False

    def read_frame(self) -> Optional[np.ndarray]:
        """
        Read a frame from the camera.

        Returns:
            Frame as numpy array, or None if failed
        """
        if self.cap is None:
            return None

        try:
            ret, frame = self.cap.read()
            return frame if ret else None
        except Exception as e:
            logging.error(f"Error reading frame: {e}")
            return None

    def save_frame(self, frame: np.ndarray, filepath: str) -> bool:
        """
        Save a frame to file.

        Args:
            frame: Image frame to save
            filepath: Path to save the image

        Returns:
            True if successful, False otherwise
        """
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            return cv2.imwrite(filepath, frame)
        except Exception as e:
            logging.error(f"Error saving frame: {e}")
            return False

    def release(self) -> None:
        """Release the camera capture resources."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def capture_images_interactive(
        self,
        save_dir: str,
        start_index: int = 0,
        file_prefix: str = "capture",
        file_ext: str = ".jpg",
    ) -> None:
        """
        Interactively capture images from camera with 's' key to save.

        Args:
            save_dir: Directory to save images
            start_index: Starting index for image naming
            file_prefix: Prefix for image filenames
            file_ext: File extension for images
        """
        if not self.start():
            logging.error("Failed to start camera")
            return

        os.makedirs(save_dir, exist_ok=True)

        i = start_index
        print("Press 's' to save image, 'ESC' to quit")

        try:
            while True:
                frame = self.read_frame()
                if frame is None:
                    continue

                cv2.imshow("Camera Capture", frame)
                k = cv2.waitKey(1) & 0xFF

                if k == 27:  # ESC key
                    break
                elif k == ord("s"):
                    filepath = os.path.join(
                        save_dir, f"{file_prefix}_{i:05d}{file_ext}"
                    )
                    if self.save_frame(frame, filepath):
                        print(f"Saved {filepath}")
                        i += 1
                    else:
                        logging.error(f"Failed to save {filepath}")

        finally:
            self.release()
            cv2.destroyAllWindows()

    def get_frame_size(self) -> Tuple[int, int]:
        """
        Get the current frame size.

        Returns:
            Tuple of (width, height)
        """
        if self.cap is None:
            return (self.width, self.height)

        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return (width, height)

    def set_resolution(self, width: int, height: int) -> bool:
        """
        Set camera resolution.

        Args:
            width: Desired width
            height: Desired height

        Returns:
            True if successful, False otherwise
        """
        if self.cap is None:
            return False

        try:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.width = width
            self.height = height
            return True
        except Exception as e:
            logging.error(f"Error setting resolution: {e}")
            return False


if __name__ == "__main__":
    # Example usage
    # camera = CameraCapture(device_id=0, width=1280, height=720)
    # camera.capture_images_interactive("./images", start_index=0)
    pass
