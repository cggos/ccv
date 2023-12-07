#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@Project : ccv 
@File    : img_match.py
@Site    : 
@Author  : Gavin Gao
@Date    : 7/24/22 7:40 AM 
"""

import argparse
import imtools as imt
import numpy as np
from PIL import Image
import cv2 as cv
import pydot

imlist = []


def draw_keypoints(img0, img1, kp0, kp1):
    outimg0 = cv.drawKeypoints(img0, keypoints=kp0, outImage=None)
    outimg1 = cv.drawKeypoints(img1, keypoints=kp1, outImage=None)
    outimg = np.hstack([outimg0, outimg1])
    cv.imshow("Key Points", outimg)
    cv.waitKey(0)


def draw_match(img0, img1, kp0, kp1, match):
    outimage = cv.drawMatches(img0, kp0, img1, kp1, match, outImg=None)
    cv.imshow("Match Result", outimage)
    cv.waitKey(0)


def get_good_matches(matches):
    # 计算最大距离和最小距离
    min_distance = matches[0].distance
    max_distance = matches[0].distance
    for x in matches:
        if x.distance < min_distance:
            min_distance = x.distance
        if x.distance > max_distance:
            max_distance = x.distance

    '''
        当描述子之间的距离大于两倍的最小距离时，认为匹配有误。
        但有时候最小距离会非常小，所以设置一个经验值30作为下限。
    '''
    good_match = []
    for x in matches:
        if x.distance <= max(2 * min_distance, 10):
            good_match.append(x)
    return good_match


def matches_graphviz(match_scores, out_img='out_imgmatch_graphviz.png', th=30):
    g = pydot.Dot(graph_type='graph')
    n = len(imlist)
    for i in range(n):
        for j in range(i + 1, n):
            if match_scores[i, j] > th:
                for x in [i, j]:
                    pil_im = Image.open(imlist[x])
                    pil_im.thumbnail((128, 128))
                    im_path = '/tmp/' + str(x) + '.png'
                    pil_im.save(im_path)
                    g.add_node(pydot.Node(str(x), fontcolor='transparent', shape='rectangle', image=im_path))
                g.add_edge(pydot.Edge(str(i), str(j)))
    g.write_png(out_img)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('img_dir', help='input image directory')
    parser.add_argument('-e', '--img_ext', default='png', help='image ext')
    args = parser.parse_args()

    img_dir = args.img_dir
    img_ext = args.img_ext

    global imlist
    imlist = imt.get_imlist(img_dir, "." + img_ext)
    if len(imlist) == 0:
        print("ERROR: imlist EMPTY!!!")
        exit(-1)
    n = len(imlist)

    orb = cv.ORB_create()
    bf = cv.BFMatcher(cv.NORM_HAMMING)
    match_scores = np.zeros((n, n))
    for i in range(n):
        for j in range(i, n):
            img0 = cv.imread(imlist[i])
            img1 = cv.imread(imlist[j])

            kp0 = orb.detect(img0)
            kp1 = orb.detect(img1)
            # draw_keypoints(img0, img1, kp0, kp1)

            kp0, des0 = orb.compute(img0, kp0)
            kp1, des1 = orb.compute(img1, kp1)
            matches = bf.match(des0, des1)
            good_matches = get_good_matches(matches)
            # draw_match(img0, img1, kp0, kp1, good_matches)

            match_scores[i, j] = len(good_matches)

    for i in range(n):
        for j in range(i + 1, n):
            match_scores[j, i] = match_scores[i, j]

    print(match_scores)

    matches_graphviz(match_scores)


if __name__ == "__main__":
    main()
