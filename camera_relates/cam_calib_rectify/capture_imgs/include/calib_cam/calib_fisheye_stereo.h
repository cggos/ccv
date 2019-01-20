#ifndef CAPTURE_CAM_CALIB_FISHEYE_STEREO_H
#define CAPTURE_CAM_CALIB_FISHEYE_STEREO_H

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class StereoFisheyeCalib {

public:
    StereoFisheyeCalib() {}

    StereoFisheyeCalib(cv::Size board_sz, double square_size, std::string calib_file)
            : board_sz_(board_sz)
            , square_size_(square_size)
            , calib_file_(calib_file)
            , img_size_(cv::Size(0,0)) {}

    ~StereoFisheyeCalib() {}

    void calib(const cv::Mat &img_l, const cv::Mat &img_r);

private:

    void calib() {

        std::cout << "=============== fisheye stereo calib ===============" << std::endl;

        cv::Matx33d K1, K2, R;
        cv::Vec3d T;
        cv::Vec4d D1, D2;

        int flag = 0;
        flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        flag |= cv::fisheye::CALIB_CHECK_COND;
        flag |= cv::fisheye::CALIB_FIX_SKEW;
        //flag |= cv::fisheye::CALIB_FIX_K2;
        //flag |= cv::fisheye::CALIB_FIX_K3;
        //flag |= cv::fisheye::CALIB_FIX_K4;

        cv::fisheye::stereoCalibrate(
                obj_points_, img_points_l_, img_points_r_,
                K1, D1, K2, D2, img_size_, R, T,
                flag, cv::TermCriteria(3, 12, 0));

        cv::Mat R1, R2, P1, P2, Q;
        cv::fisheye::stereoRectify(
                K1, D1, K2, D2, img_size_, R, T,
                R1, R2, P1, P2, Q,
                CV_CALIB_ZERO_DISPARITY, img_size_, 0.0, 1.1);

        cv::FileStorage fs(calib_file_, cv::FileStorage::WRITE);

        fs << "img_w" << img_size_.width;
        fs << "img_h" << img_size_.height;

        fs << "K1" << cv::Mat(K1);
        fs << "K2" << cv::Mat(K2);
        fs << "D1" << D1;
        fs << "D2" << D2;
        fs << "R"  << cv::Mat(R);
        fs << "T"  << T;

        fs << "R1" << R1;
        fs << "R2" << R2;
        fs << "P1" << P1;
        fs << "P2" << P2;
        fs << "Q"  << Q;

        fs.release();

        std::cout << "fisheye calib data saved in :\n  " << calib_file_ << std::endl;

        std::cout << "=============== calib complete ===============" << std::endl;
    }

public:
    cv::Size board_sz_;
    double square_size_;

    std::string calib_file_;

    cv::Size img_size_;

    std::vector<cv::Point2f> corners_l_;
    std::vector<cv::Point2f> corners_r_;
    std::vector<cv::Point3d> obj_pts_;

    std::vector< std::vector< cv::Point2f > > img_pts_l_;
    std::vector< std::vector< cv::Point2f > > img_pts_r_;

    std::vector< std::vector< cv::Point2d > > img_points_l_;
    std::vector< std::vector< cv::Point2d > > img_points_r_;
    std::vector< std::vector< cv::Point3d > > obj_points_;
};

#endif //CAPTURE_CAM_CALIB_FISHEYE_STEREO_H
