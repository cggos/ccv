#ifndef CAPTURE_CAM_CAPTURE_CAM_STEREO_IMP_H
#define CAPTURE_CAM_CAPTURE_CAM_STEREO_IMP_H

#include <ros/ros.h>
#include <opencv2/imgproc.hpp>

namespace capture_cam {

    class CaptureCamStereoImp {

    public:
        CaptureCamStereoImp() {}
        // CaptureCamStereoImp(ros::NodeHandle &nh) : nh_(nh) {}
        virtual void process(const cv::Mat &img_l, const cv::Mat &img_r) = 0;

    public:
        // ros::NodeHandle nh_;
        bool is_rectify_;
        cv::Mat   K1_;
        cv::Mat   K2_;
        cv::Vec4d D1_;
        cv::Vec4d D2_;
        cv::Mat R1_, R2_, P1_, P2_;
        cv::Mat img_rect_l_;
        cv::Mat img_rect_r_;
    };
}

#endif //CAPTURE_CAM_CAPTURE_CAM_STEREO_IMP_H
