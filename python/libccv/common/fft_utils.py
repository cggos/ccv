#!/usr/bin/env python
"""
FFT utility class for common frequency domain operations.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple


class FFTProcessor:
    """
    A utility class for performing common FFT (Fast Fourier Transform) operations
    on images in the frequency domain.
    """

    def __init__(self):
        """Initialize the FFTProcessor."""
        pass

    @staticmethod
    def fft2_centered(image: np.ndarray) -> np.ndarray:
        """
        Perform 2D FFT and shift the zero frequency component to the center.

        Args:
            image: Input image (grayscale)

        Returns:
            Centered FFT of the image
        """
        fft = np.fft.fft2(image)
        return np.fft.fftshift(fft)

    @staticmethod
    def ifft2_centered(fft_image: np.ndarray) -> np.ndarray:
        """
        Perform inverse 2D FFT with centered frequency components.

        Args:
            fft_image: FFT image with centered frequency components

        Returns:
            Reconstructed image
        """
        fft_shifted = np.fft.ifftshift(fft_image)
        return np.fft.ifft2(fft_shifted)

    @staticmethod
    def create_rectangular_mask(
        shape: Tuple[int, int], center_rect_size: Tuple[int, int], inverse: bool = False
    ) -> np.ndarray:
        """
        Create a rectangular mask for frequency filtering.

        Args:
            shape: Shape of the mask (rows, cols)
            center_rect_size: Size of the rectangle in the center (height, width)
            inverse: If True, creates an inverse mask (blocks the center)

        Returns:
            Binary mask
        """
        mask = np.ones(shape, np.uint8) if not inverse else np.zeros(shape, np.uint8)

        rows, cols = shape
        rect_h, rect_w = center_rect_size

        rs = int(rows / 2 - rect_h / 2)
        re = int(rows / 2 + rect_h / 2)
        cs = int(cols / 2 - rect_w / 2)
        ce = int(cols / 2 + rect_w / 2)

        if not inverse:
            mask[rs:re, cs:ce] = 0
        else:
            mask[rs:re, cs:ce] = 1

        return mask

    @staticmethod
    def apply_filter(
        image: np.ndarray, mask: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Apply a frequency domain filter to an image.

        Args:
            image: Input image (grayscale)
            mask: Binary mask for filtering

        Returns:
            Tuple of (filtered_image, magnitude_spectrum, phase_spectrum)
        """
        # Perform FFT
        fft_centered = FFTProcessor.fft2_centered(image)

        # Apply filter
        fft_filtered = fft_centered * mask

        # Reconstruct image
        img_back = FFTProcessor.ifft2_centered(fft_filtered)
        img_back = np.abs(img_back)

        # Normalize to 0-255 range
        img_back = np.uint8(
            (img_back - np.amin(img_back))
            / (np.amax(img_back) - np.amin(img_back))
            * 255
        )

        # Calculate magnitude and phase spectra
        magnitude_spectrum = np.log(np.abs(fft_filtered) + 1)
        phase_spectrum = np.angle(fft_filtered)

        return img_back, magnitude_spectrum, phase_spectrum

    @staticmethod
    def separate_amplitude_phase(
        fft_image: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Separate amplitude and phase components from FFT image.

        Args:
            fft_image: Complex FFT image

        Returns:
            Tuple of (amplitude, phase)
        """
        amplitude = np.abs(fft_image)
        phase = np.angle(fft_image)
        return amplitude, phase

    @staticmethod
    def combine_amplitude_phase(amplitude: np.ndarray, phase: np.ndarray) -> np.ndarray:
        """
        Combine amplitude and phase components to form a complex FFT image.

        Args:
            amplitude: Amplitude component
            phase: Phase component

        Returns:
            Complex FFT image
        """
        real = amplitude * np.cos(phase)
        imag = amplitude * np.sin(phase)

        complex_image = np.zeros(amplitude.shape, dtype=complex)
        complex_image.real = np.array(real)
        complex_image.imag = np.array(imag)

        return complex_image

    @staticmethod
    def plot_frequency_analysis(
        original_image: np.ndarray,
        filtered_image: np.ndarray,
        magnitude_spectrum: np.ndarray,
        phase_spectrum: np.ndarray,
        title_prefix: str = "",
    ) -> None:
        """
        Plot frequency analysis results.

        Args:
            original_image: Original image
            filtered_image: Filtered image
            magnitude_spectrum: Magnitude spectrum
            phase_spectrum: Phase spectrum
            title_prefix: Prefix for plot titles
        """
        plt.figure(figsize=(12, 8))

        plt.subplot(231)
        plt.imshow(original_image, "gray")
        plt.title(f"{title_prefix}Original")
        plt.xticks([]), plt.yticks([])

        plt.subplot(232)
        plt.imshow(magnitude_spectrum, "gray")
        plt.title(f"{title_prefix}Magnitude Spectrum")
        plt.xticks([]), plt.yticks([])

        plt.subplot(233)
        plt.imshow(phase_spectrum, "gray")
        plt.title(f"{title_prefix}Phase Spectrum")
        plt.xticks([]), plt.yticks([])

        plt.subplot(234)
        plt.imshow(filtered_image, "gray")
        plt.title(f"{title_prefix}Filtered")
        plt.xticks([]), plt.yticks([])

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    # Example usage
    # processor = FFTProcessor()
    # img = cv2.imread('path_to_image', 0)
    # mask = processor.create_rectangular_mask(img.shape, (60, 60))
    # filtered_img, mag, phase = processor.apply_filter(img, mask)
    # processor.plot_frequency_analysis(img, filtered_img, mag, phase)
    pass
