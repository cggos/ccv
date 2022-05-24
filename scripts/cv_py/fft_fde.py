#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage
from PIL import Image


def frequency_domain_entropy(im_gray):
    # calculate the fft magnitude
    img_float32 = np.float32(im_gray)
    dft = cv2.dft(img_float32, flags=cv2.DFT_COMPLEX_OUTPUT)
    dft_shift = np.fft.fftshift(dft)
    magnitude_spectrum = cv2.magnitude(dft_shift[:, :, 0], dft_shift[:, :, 1])

    # normalize
    magnitude_spectrum_normalized = magnitude_spectrum / np.sum(magnitude_spectrum)

    # frequency domain entropy (-> Entropy Based Measure of Camera Focus. Matej Kristan, Franjo Pernu. University of Ljubljana. Slovenia)
    fde = -np.sum(magnitude_spectrum_normalized * np.log(magnitude_spectrum_normalized))

    return fde


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--img", default="../../data/lena.bmp", help="the image path")
    args = parser.parse_args()

    # img = cv2.imread('lena.bmp', 0)

    img = Image.open(args.img).convert('L')
    im_gray = np.asarray(img)
    im_gray = ndimage.gaussian_filter(im_gray, 3)
    # cv2.imwrite("lena_00.png", im_gray)

    fde = frequency_domain_entropy(im_gray)

    text = "fde: {0}   (maximize this for focus)".format(np.sum(fde))
    print(text)
    img_out = cv2.cvtColor(im_gray, cv2.COLOR_GRAY2BGR)
    cv2.putText(img_out, text, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0, 0, 255), thickness=2)

    plt.figure()
    plt.imshow(img_out)
    plt.title('FDE')
    plt.xticks([]), plt.yticks([])
    plt.show()


if __name__ == "__main__":
    main()
