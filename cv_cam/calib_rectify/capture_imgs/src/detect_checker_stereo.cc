#include "detect_checker/detect_checker_stereo.h"

#include "capture_cam/capture_cam_stereo.h"

void StereoCheckerDetector::process(const cv::Mat &img_l, const cv::Mat &img_r) {
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
