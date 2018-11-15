#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from PIL import Image
from numpy import *


def get_imlist(path):
    return [os.path.join(path, f) for f in os.listdir(path) if f.endswith('.jpg')]


def imresize(im, sz):
    pil_im = Image.fromarray(uint8(im))
    return array(pil_im.resize(sz))


def histeq(im, nbr_bins=256):
    """ 对一幅图像进行直方图均衡化 """

    # 计算图像的直方图
    imhist, bins = histogram(im.flatten(), nbr_bins, normed=True)
    cdf = imhist.cumsum()  # 累积分布函数
    cdf = 255 * cdf / cdf[-1]  # 归一化
    # 使用累积分布函数的线性插值，计算新的像素值
    im2 = intern(im.flatten(), bins[:-1], cdf)
    return im2.reshape(im.shap), cdf


def compute_average(imlist):
    """ 计算图像列表的平均图像 """

    averageim = array(Image.open(imlist[0]), 'f')

    for imname in imlist[1:]:
        try:
            averageim += array(Image.open(imname))
        except:
            print imname + '...skipped'
    averageim /= len(imlist)
    return array(averageim, 'uint8')


def pca(X):
    """ 主成分分析：
        输入：矩阵X， 其中该矩阵中存储训练数据，每一行为一条训练数据
        返回：投影矩阵（按照维度的重要性排序），方差和均值 """

    num_data, dim = X.shape

    # 数据中心化
    mean_X = X.mean(axis=0)
    X = X - mean_X

    if dim > num_data:
        # PCA - 使用紧致技巧
        M = dot(X, X.T)        # 协方差矩阵
        e, EV = linalg.eigh(M)  # 特征值和特征向量
        tmp = dot(X.T, EV).T   # 紧致技巧
        V = tmp[::-1]          # 由于最后特征向量是我们所需要的，所以将其逆转
        S = sqrt(e)[::-1]      # 由于特征值是按照递增顺序排列的，所以将其逆转
        for i in range(V.shape[1]):
            V[:, i] /= S
    else:
        # PCA - 使用SVD方法
        U, S, V = linalg.svd(X)
        V = V[:num_data]  # 仅返回前num_data维的数据才合理

    # 返回 投影矩阵、方差和均值
    return V, S, mean_X
