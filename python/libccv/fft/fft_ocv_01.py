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

    rows, cols = img.shape
    mask = np.ones(img.shape, np.uint8)
    rs = int(rows / 2 - 30)
    re = int(rows / 2 + 30)
    cs = int(cols / 2 - 30)
    ce = int(cols / 2 + 30)
    mask[rs:re, cs:ce] = 0

    # =========================================
    img_f_o = np.fft.fft2(img)

    img_f_c = np.fft.fftshift(img_f_o)

    img_f_filter_c = img_f_c * mask

    img_f_filter_o = np.fft.ifftshift(img_f_filter_c)

    img_back = np.fft.ifft2(img_f_filter_o)

    # =========================================
    spectrum_o = np.log(np.abs(img_f_o))
    spectrum_c = np.log(np.abs(img_f_filter_c))

    phase_o = np.angle(img_f_o)
    phase_c = np.angle(img_f_filter_c)

    img_back = np.abs(img_back)

    # =========================================
    plt.subplot(321), plt.imshow(spectrum_o, "gray"), plt.title("origin spectrum")
    plt.subplot(322), plt.imshow(spectrum_c, "gray"), plt.title("center spectrum")
    plt.subplot(323), plt.imshow(phase_o, "gray"), plt.title("origin phase")
    plt.subplot(324), plt.imshow(phase_c, "gray"), plt.title("center phase")
    plt.subplot(325), plt.imshow(img, "gray"), plt.title("origin")
    plt.subplot(326), plt.imshow(img_back, "gray"), plt.title("back")

    plt.show()


if __name__ == "__main__":
    main()
