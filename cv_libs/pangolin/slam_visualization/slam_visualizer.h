#ifndef SLAM_VISUALIZER_H_
#define SLAM_VISUALIZER_H_

#include <pangolin/pangolin.h>

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "custom_struct.h"

using namespace std;

class SlamVisualizer {
   public:
    SlamVisualizer(int width = 752, int height = 480) : WIN_WIDTH_(width), WIN_HEIGHT_(height) {}
    ~SlamVisualizer() {}

    void initDraw();

    void activeAllView();

    void drawCubeTest();

    void drawCamWithPose(Eigen::Vector3d &pos, Eigen::Quaterniond &quat);

    void drawTraj(vector<Eigen::Vector3d> &traj);
    
    void drawCam(const float scale = 1.);

    void drawCoordinate();

    void displayImg(cv::Mat &originImg, cv::Mat &trackImg);

    void displayData(Eigen::Vector3d &pos, Eigen::Quaterniond &quat);

    void registerUICallback();

   private:
    pangolin::OpenGlRenderState s_cam_;
    pangolin::View d_cam_, d_img_, d_track_;
    pangolin::GlTexture imageTexture_, trackTexture_;
    pangolin::DataLog pose_log_;

    // 存储ui面板的控件对象
    std::vector<pangolin::Var<bool>> ui_set_;
    // 存储data面板的控件对象
    std::vector<pangolin::Var<VecXd>> data_set_;
    // 是否显示相机
    bool camera_visible_ = true;
    // 是否显示轨迹
    bool traj_visible_ = true;
    // 是否显示参考坐标系
    bool coordinate_visible_ = true;
    // 是否显示图像
    bool img_visible_ = true;
    // 窗口尺寸
    int WIN_WIDTH_;
    int WIN_HEIGHT_;
};

#endif