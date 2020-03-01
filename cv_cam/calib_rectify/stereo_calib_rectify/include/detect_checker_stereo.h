#ifndef CAPTURE_CAM_DETECT_CHECKER_STEREO_H
#define CAPTURE_CAM_DETECT_CHECKER_STEREO_H

#include "capture_cam_stereo_imp.h"

#include "calib_fisheye_stereo.h"

using namespace capture_cam;

class StereoCheckerDetector : public CaptureCamStereoImp {

public:
    StereoCheckerDetector(ros::NodeHandle &nh, std::string param_dir) {
        int corner_num_x;
        int corner_num_y;
        double square_size;

        nh.param("corner_num_x", corner_num_x, 11);
        nh.param("corner_num_y", corner_num_y, 7);
        nh.param("square_size",  square_size, 0.03);
        nh.param("is_rectify",   is_rectify_, false);

        cv::Size corner_num = cv::Size(corner_num_x, corner_num_y);

        std::string fisheye_calib_file = param_dir + "/fisheye_calib.yaml";

        calib_stereo_fisheye_ = StereoFisheyeCalib(corner_num, square_size, fisheye_calib_file);
    }

    void process(const cv::Mat &img_l, const cv::Mat &img_r) {
        if (!is_rectify_)
            calib_stereo_fisheye_.calib(img_l, img_r);
        else {
            calib_stereo_fisheye_.rectify(img_l, img_r, img_rect_l_, img_rect_r_);
            K1_ = calib_stereo_fisheye_.K1_;
            K2_ = calib_stereo_fisheye_.K2_;
            D1_ = calib_stereo_fisheye_.D1_;
            D2_ = calib_stereo_fisheye_.D2_;
            R1_ = calib_stereo_fisheye_.R1_;
            R2_ = calib_stereo_fisheye_.R2_;
            P1_ = calib_stereo_fisheye_.P1_;
            P2_ = calib_stereo_fisheye_.P2_;
        }
    }

private:
    StereoFisheyeCalib calib_stereo_fisheye_;
};

#endif //CAPTURE_CAM_DETECT_CHECKER_STEREO_H
