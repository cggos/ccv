//
// Created by cg on 4/21/19.
//

#ifndef CGOCV_IMAGE_OCV_H
#define CGOCV_IMAGE_OCV_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace cg {

    class ImageOCV {
    public:
        /// pseudocolor / false color a grayscale image using OpenCV’s predefined colormaps
        static void get_colormap_ocv(const cv::Mat &mat_in, cv::Mat &color_map);
    };
}

#endif //CGOCV_IMAGE_OCV_H
