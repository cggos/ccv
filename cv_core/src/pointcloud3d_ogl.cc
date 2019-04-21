//
// Created by cg on 4/21/19.
//

#include "cgocv/pointcloud3d_ogl.h"

#include <unistd.h>
#include <iostream>
#include <pangolin/pangolin.h>

namespace cg {
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
}

