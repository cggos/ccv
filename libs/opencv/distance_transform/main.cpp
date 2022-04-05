// Copyright https://mangoroom.cn
// License(MIT)
// Author:mango
// image proccess algorithm  | 图像处理算法
// this is main.cpp

#include "distance_transform.h"

int main() {
  cv::Mat src = cv::Mat::zeros(cv::Size(600, 400), CV_8UC1);

  for (size_t i = 100; i < 180; i++) {
    for (size_t j = 200; j < 400; j++) {
      src.at<uchar>(i, j) = 255;
    }
  }

  cv::Mat dst = src.clone();
  imageprocess::DistanceTransform(src, dst);
  normalize(dst, dst, 0, 255, cv::NORM_MINMAX);

//   // opencv
//   cv::threshold(src, src, 100, 255, cv::THRESH_BINARY);
//   cv::distanceTransform(src, dst, cv::DIST_L1, cv::DIST_MASK_PRECISE);
//   normalize(dst, dst, 0, 1, cv::NORM_MINMAX);

  cv::imshow("src", src);
  cv::imshow("dst", dst);

  cv::imwrite("dst.jpg", dst);
  cv::waitKey(0);

  return 0;
}
