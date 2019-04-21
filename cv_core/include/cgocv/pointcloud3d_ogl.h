//
// Created by cg on 4/21/19.
//

#ifndef CGOCV_POINTCLOUD3D_OGL_H
#define CGOCV_POINTCLOUD3D_OGL_H

#include <Eigen/Core>

namespace cg {
    class PointCloud3D {
    public:
        void show_pointcloud(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud);
    };
}

#endif //CGOCV_POINTCLOUD3D_OGL_H
