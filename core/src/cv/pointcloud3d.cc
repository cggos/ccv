//
// Created by cg on 4/21/19.
//

#include "ccv/cv/pointcloud3d.h"

#include <unistd.h>
#include <iostream>

#ifdef WITH_GL
#include <pangolin/pangolin.h>
#endif

namespace cg {

#ifdef WITH_PCL
    void PointCloud3D::depth_to_pointcloud(
            const cv::Mat &mat_depth, const cv::Mat &mat_left,
            const StereoCameraModel &camera_model,
            pcl::PointCloud<pcl::PointXYZRGB> &point_cloud) {

        point_cloud.height = (uint32_t) mat_depth.rows;
        point_cloud.width  = (uint32_t) mat_depth.cols;
        point_cloud.is_dense = false;
        point_cloud.resize(point_cloud.height * point_cloud.width);

        for (int h = 0; h < (int) mat_depth.rows; h++) {
            for (int w = 0; w < (int) mat_depth.cols; w++) {

                pcl::PointXYZRGB &pt = point_cloud.at(h * point_cloud.width + w);

                switch(mat_left.channels()) {
                    case 1:
                    {
                        unsigned char v = mat_left.at<unsigned char>(h, w);
                        pt.b = v;
                        pt.g = v;
                        pt.r = v;
                    }
                        break;
                    case 3:
                    {
                        cv::Vec3b v = mat_left.at<cv::Vec3b>(h, w);
                        pt.b = v[0];
                        pt.g = v[1];
                        pt.r = v[2];
                    }
                        break;
                }

                float depth = 0.f;
                switch (mat_depth.type()) {
                    case CV_16UC1: // unit is mm
                        depth = float(mat_depth.at<unsigned short>(h,w));
                        depth *= 0.001f;
                        break;
                    case CV_32FC1: // unit is meter
                        depth = mat_depth.at<float>(h,w);
                        break;
                }

                double W = depth / camera_model.left.fx;
                if (std::isfinite(depth) && depth >= 0) {
                    pt.x = float((cv::Point2f(w, h).x - camera_model.left.cx) * W);
                    pt.y = float((cv::Point2f(w, h).y - camera_model.left.cy) * W);
                    pt.z = depth;
                } else
                    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    void PointCloud3D::generate_pointcloud(
            const cv::Mat &mat_l, const cv::Mat &mat_disp,
            std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pointcloud) {

        double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
        double d = 0.573;

        for (int v = 0; v < mat_l.rows; v++) {
            for (int u = 0; u < mat_l.cols; u++) {
                Eigen::Vector4d point(0, 0, 0, mat_l.at<uchar>(v, u) / 255.0);
                point[2] = fx * d / mat_disp.at<uchar>(v, u);
                point[0] = (u - cx) / fx * point[2];
                point[1] = (v - cy) / fy * point[2];
                pointcloud.push_back(point);
            }
        }
    }
#endif    

#ifdef WITH_GL
    void PointCloud3D::show_pointcloud(
            const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud) {

        if (pointcloud.empty()) {
            std::cerr << "Point cloud is empty!" << std::endl;
            return;
        }

        pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
        );

        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        while (pangolin::ShouldQuit() == false) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            glPointSize(2);
            glBegin(GL_POINTS);
            for (auto &p: pointcloud) {
                glColor3f(p[3], p[3], p[3]);
                glVertex3d(p[0], p[1], p[2]);
            }
            glEnd();
            pangolin::FinishFrame();
            usleep(5000);   // sleep 5 ms
        }
        return;
    }
#endif

}

