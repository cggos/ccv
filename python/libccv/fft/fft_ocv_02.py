#!/usr/bin/env python

import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import logging
from libccv.common.logger import init_logger


def main():
    init_logger()
    parser = argparse.ArgumentParser(description="FFT processing with OpenCV")
    parser.add_argument(
        "--image", default="../../data/lena.bmp", help="Path to the input image"
    )
    args = parser.parse_args()

    # Check if image file exists
    if not os.path.exists(args.image):
        logging.error(f"Image file {args.image} not found.")
        return

    img = cv2.imread(args.image, 0)

    if img is None:
        logging.error(f"Could not load image from {args.image}")
        return

    img_f_o = np.fft.fft2(img)
    img_f_c = np.fft.fftshift(img_f_o)

    img_amplitude = np.abs(img_f_c)
    img_phase = np.angle(img_f_c)

    img_real = img_amplitude * np.cos(img_phase)
    img_imag = img_amplitude * np.sin(img_phase)

    img_new_complex = np.zeros(img.shape, dtype=complex)
    img_new_complex.real = np.array(img_real)
    img_new_complex.imag = np.array(img_imag)

    img_new_f_o = np.fft.ifftshift(img_new_complex)

    img_new = np.fft.ifft2(img_new_f_o)
    img_new = np.abs(img_new)

    img_back = (img_new - np.amin(img_new)) / (np.amax(img_new) - np.amin(img_new))

    plt.figure()
    plt.imshow(img_back, "gray"), plt.title("amplitude + phase")
    plt.xticks([]), plt.yticks([])
    plt.show()


if __name__ == "__main__":
    main()
