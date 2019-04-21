//
// Created by cg on 4/21/19.
//

#include "cgocv/image_ocv.h"

#include <iostream>

namespace cg {

    void ImageOCV::get_colormap_ocv(const cv::Mat &mat_in, cv::Mat &color_map) {

        double min, max;
        cv::minMaxLoc(mat_in, &min, &max);

        cv::Mat mat_scaled;
        if(min != max)
            mat_in.convertTo(mat_scaled, CV_8UC1, 255.0/(max-min), -255.0*min/(max-min));

        cv::applyColorMap(mat_scaled, color_map, cv::COLORMAP_JET);
    }
}

