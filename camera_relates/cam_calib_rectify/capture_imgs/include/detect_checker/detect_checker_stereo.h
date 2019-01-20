#ifndef CAPTURE_CAM_DETECT_CHECKER_STEREO_H
#define CAPTURE_CAM_DETECT_CHECKER_STEREO_H

#include "capture_cam/capture_cam_stereo_imp.h"

#include "calib_cam/calib_fisheye_stereo.h"

using namespace capture_cam;

class StereoCheckerDetector : public CaptureCamStereoImp {

public:
    StereoCheckerDetector(ros::NodeHandle &nh, std::string param_dir) : CaptureCamStereoImp(nh) {
        int corner_num_x;
        int corner_num_y;
        double square_size;

        nh_.param("corner_num_x", corner_num_x, 11);
        nh_.param("corner_num_y", corner_num_y, 7);
        nh_.param("square_size",  square_size, 0.03);

        cv::Size corner_num = cv::Size(corner_num_x, corner_num_y);

        std::string fisheye_calib_file = param_dir + "/fisheye_calib.yaml";

        calib_stereo_fisheye_ = StereoFisheyeCalib(corner_num, square_size, fisheye_calib_file);
    }

    void process(const cv::Mat &img_l, const cv::Mat &img_r);

private:
    StereoFisheyeCalib calib_stereo_fisheye_;

};

#endif //CAPTURE_CAM_DETECT_CHECKER_STEREO_H
