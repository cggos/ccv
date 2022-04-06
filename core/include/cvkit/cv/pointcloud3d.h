//
// Created by cg on 4/21/19.
//

#ifndef CGOCV_POINTCLOUD3D_OGL_H
#define CGOCV_POINTCLOUD3D_OGL_H

#include <vector>
#include <Eigen/Core>

#ifdef WITH_PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#endif

#include <opencv2/core/core.hpp>

#include "cvkit/cv/camera.h"

namespace cg {
    class PointCloud3D {
    public:

#ifdef WITH_PCL
        static void depth_to_pointcloud(const cv::Mat &mat_depth, const cv::Mat &mat_left, 
                                        const StereoCameraModel &camera_model,
                                        pcl::PointCloud<pcl::PointXYZRGB> &point_cloud);

        static void generate_pointcloud(const cv::Mat &mat_l, const cv::Mat &mat_disp,
                                 std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pointcloud);
#endif                                

#ifdef WITH_GL
        static void show_pointcloud(
                const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud);
#endif    
    };
}

#endif //CGOCV_POINTCLOUD3D_OGL_H
