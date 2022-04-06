#ifndef CGOCV_STEREO_CAMERA_H
#define CGOCV_STEREO_CAMERA_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cvkit/cv/camera.h"

namespace cg {

    class StereoCamera {

    public:
      void compute_disparity_map(const cv::Mat &mat_l, const cv::Mat &mat_r, cv::Mat &mat_disp);

      void disparity_to_depth_map(const cv::Mat &mat_disp, cv::Mat &mat_depth);

    public:
      StereoCameraModel camera_model_;
    };
};

#endif //CGOCV_STEREO_CAMERA_H
