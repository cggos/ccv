//
// Created by cg on 4/21/19.
//

#ifndef CCV_CV_IMAGE_OCV_H
#define CCV_CV_IMAGE_OCV_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cg {

class ImageOCV {
 public:
  /// pseudocolor / false color a grayscale image using OpenCV’s predefined colormaps
  static void get_colormap_ocv(const cv::Mat &mat_in,
                               cv::Mat &color_map,
                               cv::ColormapTypes colortype = cv::COLORMAP_JET);
};
}  // namespace cg

#endif  // CCV_CV_IMAGE_OCV_H
