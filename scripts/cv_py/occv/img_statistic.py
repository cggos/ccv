#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
from PIL import Image
from scipy import ndimage
import numpy as np
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('img_in', help='input image')
    args = parser.parse_args()

    img_path = args.img_in
    img = Image.open(img_path).convert('L')
    im = np.asarray(img)

    im_blur = ndimage.gaussian_filter(im, 4)

    a = np.corrcoef(im.flatten(), im_blur.flatten())

    print(a)

    plt.subplot(121), plt.imshow(im, plt.cm.gray), plt.title('origin gray')
    plt.xticks([]), plt.yticks([])
    plt.subplot(122), plt.imshow(im_blur, plt.cm.gray), plt.title('gauss blur')
    plt.xticks([]), plt.yticks([])

    plt.show()


if __name__ == "__main__":
    main()
