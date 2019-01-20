#ifndef CAPTURE_IMGS_DETECT_CHARUCO_STEREO_H
#define CAPTURE_IMGS_DETECT_CHARUCO_STEREO_H

#include "capture_cam/capture_cam_stereo_imp.h"
#include "detect_charuco/detect_charuco.h"

using namespace capture_cam;

class StereoCharucoDetector : public CaptureCamStereoImp {

public:
    StereoCharucoDetector(ros::NodeHandle &nh, std::string param_dir) : CaptureCamStereoImp(nh) {
        int id_dictionary = 10;
        int squares_x;
        int squares_y;
        float square_length;
        float marker_length;
        nh_.param("squares_x", squares_x, 7);
        nh_.param("squares_y", squares_y, 11);
        nh_.param("square_length", square_length, 0.021f);
        nh_.param("marker_length", marker_length, 0.014f);
        charuco_detector_r_ = charuco_detector_l_ =
                CharucoDetector(id_dictionary, squares_x, squares_y, square_length, marker_length);
    }

    void process(const cv::Mat &img_l, const cv::Mat &img_r);

private:
    CharucoDetector charuco_detector_l_;
    CharucoDetector charuco_detector_r_;

};

#endif //CAPTURE_IMGS_DETECT_CHARUCO_STEREO_H
