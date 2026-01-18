import cv2
import threading

class Camera:
    """
    Class to handle camera operations.
    Using a singleton pattern or simple management to avoid multiple access conflicts
    is often good, but here we keep it simple.
    """
    def __init__(self, camera_id=0):
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {camera_id}")

    def get_frame(self):
        """Captures a single frame from the camera."""
        if not self.cap.isOpened():
            return None
        
        success, frame = self.cap.read()
        if not success:
            return None
        return frame

    def release(self):
        if self.cap.isOpened():
            self.cap.release()
