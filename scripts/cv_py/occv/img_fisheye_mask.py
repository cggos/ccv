#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：cv_py 
@File    ：fisheye_mask.py
@Author  ：Hongchen Gao
@Date    ：3/9/22 4:58 PM 
'''

import cv2
import numpy as np
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('img_in', help='input image')
    args = parser.parse_args()

    img_path = args.img_in
    img = cv2.imread(img_path)
    print(img.shape)
    rows, cols = img.shape[:2]

    # rows = 800
    # cols = 848

    xc = cols // 2
    yc = rows // 2
    radius = rows // 2
    radius -= 10

    # draw filled circles in white on black background as masks
    # mask = np.zeros_like(img)
    mask = np.zeros((rows, cols), np.uint8)
    cv2.circle(mask, (xc,yc), radius, 255, -1)

    # put mask into alpha channel of input
    # result = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
    # result[:, :, 1] = mask[:,:,0]

    # result = cv2.add(img, mask)

    result = cv2.bitwise_and(img, img, mask=mask)

    # plt.figure()
    # plt.imshow(masked)
    # plt.title('circle mask')
    # plt.show()

    cv2.imwrite("out-fisheye_mask_848_800.png", mask)

    cv2.imshow('masked image', result)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()
