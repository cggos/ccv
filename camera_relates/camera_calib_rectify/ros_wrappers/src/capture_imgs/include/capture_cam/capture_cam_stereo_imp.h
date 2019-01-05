#ifndef CAPTURE_CAM_CAPTURE_CAM_STEREO_IMP_H
#define CAPTURE_CAM_CAPTURE_CAM_STEREO_IMP_H

#include <ros/ros.h>
#include <opencv2/imgproc.hpp>

namespace capture_cam {

    class CaptureCamStereoImp {
    public:
        CaptureCamStereoImp() {}
        CaptureCamStereoImp(ros::NodeHandle &nh) : nh_(nh) {}
        virtual void process(const cv::Mat &img_l, const cv::Mat &img_r) = 0;
    public:
        ros::NodeHandle nh_;
    };
}

#endif //CAPTURE_CAM_CAPTURE_CAM_STEREO_IMP_H
