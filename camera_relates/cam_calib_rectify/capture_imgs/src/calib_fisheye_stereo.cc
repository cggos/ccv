#include "calib_cam/calib_fisheye_stereo.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void StereoFisheyeCalib::calib(const cv::Mat &img_l, const cv::Mat &img_r) {

    static bool is_get_size = false;
    if(!is_get_size) {
        img_size_ = img_l.size();
        is_get_size = true;
    }

    corners_l_.clear();
    corners_r_.clear();
    corners_l_.reserve(board_sz_.area());
    corners_r_.reserve(board_sz_.area());

    bool found_l = cv::findChessboardCorners(img_l, board_sz_, corners_l_, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    bool found_r = cv::findChessboardCorners(img_r, board_sz_, corners_r_, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    cv::Mat mat_color_l(img_l.size(), CV_8UC3);
    cv::Mat mat_color_r(img_r.size(), CV_8UC3);

    cv::cvtColor(img_l, mat_color_l, CV_GRAY2BGR);
    cv::cvtColor(img_r, mat_color_r, CV_GRAY2BGR);

    char key = (char)cv::waitKey(10);

    if(found_l && found_r)
    {
        cv::cornerSubPix(img_l, corners_l_, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        cv::cornerSubPix(img_r, corners_r_, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

        cv::drawChessboardCorners(mat_color_l, board_sz_, corners_l_, found_l);
        cv::drawChessboardCorners(mat_color_r, board_sz_, corners_r_, found_r);

        if(key == 's') {
            static int n_frame = 0;

            obj_pts_.clear();
            obj_pts_.reserve(board_sz_.area());
            for( int i = 0; i < board_sz_.height; ++i )
                for( int j = 0; j < board_sz_.width; ++j )
                    obj_pts_.push_back(cv::Point3d((float)j * square_size_, (float)i * square_size_, 0.0));

            std::cout << "found and save image pair " << n_frame << std::endl;

            img_pts_l_.push_back(corners_l_);
            img_pts_r_.push_back(corners_r_);
            obj_points_.push_back(obj_pts_);

            n_frame++;
        }
    }

    if(key == 'c') {

        for (int i = 0; i < img_pts_l_.size(); i++) {
            std::vector< cv::Point2d > v1, v2;
            for (int j = 0; j < img_pts_l_[i].size(); j++) {
                v1.push_back(cv::Point2d((double)img_pts_l_[i][j].x, (double)img_pts_l_[i][j].y));
                v2.push_back(cv::Point2d((double)img_pts_r_[i][j].x, (double)img_pts_r_[i][j].y));
            }
            img_points_l_.push_back(v1);
            img_points_r_.push_back(v2);
        }

        calib();
    }

    cv::Mat img_concat;
    cv::hconcat(mat_color_l, mat_color_r, img_concat);

    putText(img_concat, "Press 's' to add current frame, 'c' to finish and calibrate",
            cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

    cv::imshow("Stereo Checker Detector", img_concat);
}

