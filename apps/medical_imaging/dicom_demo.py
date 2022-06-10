#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy as np
import matplotlib.pyplot as plt

# pip install --user pydicom
import pydicom as pdicom


def load_scan2(dir):
    ds_list = []
    for dir_name, subdir_lst, file_lst in os.walk(dir):
        for file_name in file_lst:
            ds_list.append(os.path.join(dir_name, file_name))
    return ds_list


def get_dcm_info(dcm):
    info = {}
    # 通过字典关键字来获取图像的数据元信息（当然也可以根据TAG号）
    # 这里获取几种常用信息
    info["UID"] = dcm.SOPInstanceUID  # 获取图像唯一标识符UID
    info["PatientID"] = dcm.PatientID  # 患者ID
    info["PatientName"] = dcm.PatientName  # 患者姓名
    # info["PatientBirthData"] = dcm.PatientBirthData  # 患者出生日期
    info["PatientAge"] = dcm.PatientAge  # 患者年龄
    info['PatientSex'] = dcm.PatientSex  # 患者性别
    info['StudyID'] = dcm.StudyID  # 检查ID
    info['StudyDate'] = dcm.StudyDate  # 检查日期
    info['StudyTime'] = dcm.StudyTime  # 检查时间
    info['InstitutionName'] = dcm.InstitutionName  # 机构名称
    info['Manufacturer'] = dcm.Manufacturer  # 设备制造商
    info['StudyDescription'] = dcm.StudyDescription  # 检查项目描述
    return info


data_dir = '/home/cg/Downloads/SE10/'

# files = os.listdir(data_dir)
# files.sort()
# print(files)

dcm_lst = load_scan2(data_dir)

dcm_ref = pdicom.read_file(dcm_lst[0])

print("图片中存在的属性： ", dcm_ref.dir("pat"))

print(dcm_ref)

print("dcm info: {}".format(get_dcm_info(dcm_ref)))

print("Patient Position", dcm_ref.PatientPosition)
# print("病人方位： ", dcm_ref.PatientOrientation)
print("图像病人方向： ", dcm_ref.ImageOrientationPatient)
print("图像病人位置： ", dcm_ref.ImagePositionPatient)

pixel_dims = (int(dcm_ref.Rows), int(dcm_ref.Columns), len(dcm_lst))
print("DICOM img 长、宽、层数: {}".format(pixel_dims))

# 得到spacing值 (mm为单位)
# PixelSpacing - 每个像素点实际的长度与宽度,单位(mm)
# SliceThickness - 每层切片的厚度,单位(mm)
pixel_spacing = (float(dcm_ref.PixelSpacing[0]), float(dcm_ref.PixelSpacing[1]), float(dcm_ref.SliceThickness))

print("pixel_spacing: {}".format(pixel_spacing))

x = np.arange(0.0, (pixel_dims[0] + 1) * pixel_spacing[0], pixel_spacing[0])  # 0到（第一个维数加一*像素间的间隔），步长为constpixelSpacing
y = np.arange(0.0, (pixel_dims[1] + 1) * pixel_spacing[1], pixel_spacing[1])
z = np.arange(0.0, (pixel_dims[2] + 1) * pixel_spacing[2], pixel_spacing[2])

dcm_data = np.zeros(pixel_dims, dtype=dcm_ref.pixel_array.dtype)

for file_name in dcm_lst:
    dcm = pdicom.read_file(file_name)
    dcm_data[:, :, dcm_lst.index(file_name)] = dcm.pixel_array

plt.figure()
plt.title("dcm0")
plt.imshow(dcm_ref.pixel_array, cmap=plt.cm.bone)

plt.figure()  # dpi=500
plt.title("轴状面")
plt.axes().set_aspect('equal')
plt.set_cmap(plt.gray())
plt.pcolormesh(x, y, np.flipud(dcm_data[:, :, 10]))
# plt.imshow(dcm_data[:,:,88])

plt.figure()
plt.title("冠状面")
plt.axes().set_aspect('equal', 'datalim')
plt.set_cmap(plt.gray())
plt.imshow(dcm_data[:, 90, :])

plt.show()
