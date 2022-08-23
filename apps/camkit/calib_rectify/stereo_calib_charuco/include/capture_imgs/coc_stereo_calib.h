#ifndef CAPTURE_IMGS_COC_STEREO_CALIB_H
#define CAPTURE_IMGS_COC_STEREO_CALIB_H

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>

#include "camodocal/camera_models/Camera.h"
#include "camodocal/calib/StereoCameraCalibration.h"
#include "camodocal/camera_models/EquidistantCamera.h"

class CocStereoCalib {
public:
    CocStereoCalib() {}

    CocStereoCalib(cv::Size boardSize, std::string str_param_dir)
    : board_size_(boardSize), param_dir_(str_param_dir) {}

    ~CocStereoCalib() {}

    inline void init(const cv::Size &imageSize) {
        camodocal::Camera::ModelType modeType = camodocal::Camera::KANNALA_BRANDT;
        coc_stereo_calib_ = camodocal::StereoCameraCalibration(modeType, "left", "right", imageSize, board_size_);
        coc_stereo_calib_.setVerbose(true);
    }

    inline void add_calib_data(
            std::vector<std::vector<int> > ids_l,
            std::vector<std::vector<cv::Point2f>> img_corners_l,
            std::vector<std::vector<cv::Point3f>> obj_corners_l,
            std::vector<std::vector<int> > ids_r,
            std::vector<std::vector<cv::Point2f>> img_corners_r,
            std::vector<std::vector<cv::Point3f>> obj_corners_r) {
        coc_stereo_calib_.addCalibDataAll(ids_l, img_corners_l, obj_corners_l, ids_r, img_corners_r, obj_corners_r);
    }

    inline void calib() {

        if(!coc_stereo_calib_.calibrate()) {
            std::cerr << "calibrate failture, restart calibrate..." << std::endl;
            coc_stereo_calib_.clear();
            return;
        }
        coc_stereo_calib_.writeParams(param_dir_);

        coc_stereo_calib_.clear();

        std::cout << "calibrate end, and save param files to: \n " << param_dir_ << std::endl;
    }

private:
    camodocal::StereoCameraCalibration coc_stereo_calib_;
    cv::Size board_size_;
    std::string param_dir_;
};

#endif //CAPTURE_IMGS_COC_STEREO_CALIB_H
