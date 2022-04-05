#pragma once

// Copyright https://mangoroom.cn
// License(MIT)
// Author:mango
// distance transfer | 距离变换
// this is distance_transform.h

#include <opencv2/opencv.hpp>

namespace imageprocess {
// Distance transform function:距离变换函数
void DistanceTransform(const cv::Mat& src_image, cv::Mat& dst_image);

// Calculate City block distance： 计算城市街区距离
int D4(const int& x1, const int& x2, const int& y1, const int& y2);

// Calculate chessboard distance：计算棋盘距离
int D8(const int& x1, const int& x2, const int& y1, const int& y2);

}  // namespace imageprocess