#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import imtools
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('img_in', help='input image')
    args = parser.parse_args()

    img_path = args.img_in
    img_raw = cv2.imread(img_path)

    bright0 = imtools.brightness_by_gray(img_raw)
    bright1 = imtools.brightness_by_hsv(img_raw)

    print("img bright: gray {}, hsv {}".format(bright0, bright1))


if __name__ == "__main__":
    main()
