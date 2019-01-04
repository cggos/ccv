#ifndef CGOCV_STEREO_CAMERA_H
#define CGOCV_STEREO_CAMERA_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "cgocv/camera.h"

namespace cg {

    class StereoCamera {

      struct StereoCameraModel {
         float baseline;
         CameraModel left;
         CameraModel right;
      };

    public:
        void compute_depth_map(const cv::Mat &mat_left, const cv::Mat &mat_right,
                cv::Mat &mat_depth);

        inline void generate_colormap(const cv::Mat &input_mat, cv::Mat *color_map) {
            double min, max;
            cv::minMaxIdx(input_mat, &min, &max);
            cv::Mat scaled_input_mat;
            input_mat.convertTo(scaled_input_mat, CV_8UC1, 255 / (max - min), -min);
            cv::applyColorMap(scaled_input_mat, *color_map, cv::COLORMAP_RAINBOW);
        }

        void depth_to_pointcloud(const cv::Mat &mat_depth, const cv::Mat &mat_left,
                pcl::PointCloud<pcl::PointXYZRGB> &point_cloud);
    public:
        StereoCameraModel camera_model_;
    };
};

#endif //CGOCV_STEREO_CAMERA_H
