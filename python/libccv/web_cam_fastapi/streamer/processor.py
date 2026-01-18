import cv2
import numpy as np

class ImageProcessor:
    """Base class for image processors."""
    def process(self, frame: np.ndarray) -> np.ndarray:
        raise NotImplementedError("Process method must be implemented")

class GaussianBlurProcessor(ImageProcessor):
    """Applies Gaussian Blur to the image."""
    def __init__(self, kernel_size: tuple = (21, 21), sigma: float = 0):
        self.kernel_size = kernel_size
        self.sigma = sigma

    def process(self, frame: np.ndarray) -> np.ndarray:
        if frame is None:
            return None
        return cv2.GaussianBlur(frame, self.kernel_size, self.sigma)
