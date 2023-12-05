#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import cv2
import imtools


def main():
    img_path = sys.argv[1]
    img_raw = cv2.imread(img_path)

    bright0 = imtools.brightness_by_gray(img_raw)
    bright1 = imtools.brightness_by_hsv(img_raw)

    print("img bright: gray {}, hsv {}".format(bright0, bright1))


if __name__ == "__main__":
    main()
