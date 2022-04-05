#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import pydicom as pdicom

folder_input = '/home/cg/Downloads/SE10/'

files = os.listdir(folder_input)
files.sort()

# print files

dcm_lst = []

def load_scan2(dir):
  for dir_name, subdir_lst, file_lst in os.walk(dir):
    for file_name in file_lst:
      dcm_lst.append(os.path.join(dir_name, file_name))

load_scan2(folder_input)

dcm_ref = pdicom.read_file(dcm_lst[0])

plt.imshow(dcm_ref.pixel_array, cmap=plt.cm.bone)
plt.show()

print dcm_ref.Rows
print dcm_ref.Columns
print len(dcm_lst)

# 建立三维数组,分别记录长、宽、层数(也就是dicom数据个数)
pixel_dims = (int(dcm_ref.Rows), int(dcm_ref.Columns), len(dcm_lst))

# 得到spacing值 (mm为单位)
# PixelSpacing - 每个像素点实际的长度与宽度,单位(mm)
# SliceThickness - 每层切片的厚度,单位(mm)
pixel_spacing = (float(dcm_ref.PixelSpacing[0]), float(dcm_ref.PixelSpacing[1]), float(dcm_ref.SliceThickness))

print pixel_spacing

x = np.arange(0.0, (pixel_dims[0]+1)*pixel_spacing[0], pixel_spacing[0]) # 0到（第一个维数加一*像素间的间隔），步长为constpixelSpacing
y = np.arange(0.0, (pixel_dims[1]+1)*pixel_spacing[1], pixel_spacing[1])
z = np.arange(0.0, (pixel_dims[2]+1)*pixel_spacing[2], pixel_spacing[2])

dcm_data = np.zeros(pixel_dims, dtype=dcm_ref.pixel_array.dtype)

for file_name in dcm_lst:
  dcm = pdicom.read_file(file_name)
  dcm_data[:,:,dcm_lst.index(file_name)] = dcm.pixel_array

# 轴状面显示
# plt.figure(dpi=500)
plt.axes().set_aspect('equal')
plt.set_cmap(plt.gray())
plt.pcolormesh(x,y,np.flipud(dcm_data[:,:,88]))
# plt.imshow(dcm_data[:,:,88])
plt.show()

# 冠状面显示
# plt.figure(dpi=500)
plt.axes().set_aspect('equal', 'datalim')
plt.set_cmap(plt.gray())
plt.imshow(dcm_data[:,90,:])
plt.show()
