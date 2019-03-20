#ifndef CAPTURE_CAM_CALIB_FISHEYE_STEREO_H
#define CAPTURE_CAM_CALIB_FISHEYE_STEREO_H

#include <iostream>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;

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

    void rectify(const cv::Mat &img_l, const cv::Mat &img_r);

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

        cv::FileStorage fs(calib_file_, cv::FileStorage::WRITE);

        fs << "img_w" << img_size_.width;
        fs << "img_h" << img_size_.height;

        fs << "K1" << cv::Mat(K1);
        fs << "K2" << cv::Mat(K2);
        fs << "D1" << D1;
        fs << "D2" << D2;
        fs << "R"  << cv::Mat(R);
        fs << "T"  << T;

        fs.release();

        std::cout << "fisheye calib data saved in :\n  " << calib_file_ << std::endl;

        std::cout << "=============== calib complete ===============" << std::endl;
    }

    void get_rect_map() {

        cv::Mat   K1;
        cv::Mat   K2;
        cv::Vec4d D1;
        cv::Vec4d D2;
        cv::Mat   R;
        cv::Vec3d t;

        cv::Size img_size;

        cv::FileStorage fs(calib_file_,cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "Failed to open calibration parameter file." << std::endl;
            exit(1);
        }

        fs["K1"] >> K1;
        fs["K2"] >> K2;
        fs["D1"] >> D1;
        fs["D2"] >> D2;
        fs["R"]  >> R;
        fs["T"]  >> t;

        fs["img_w"] >> img_size.width;
        fs["img_h"] >> img_size.height;

        fs.release();

        cv::Size new_size = img_size ;//+ cv::Size(300, 400);

        cv::Mat R1, R2, P1, P2, Q;
        cv::fisheye::stereoRectify(
                K1, D1, K2, D2, img_size, R, t,
                R1, R2, P1, P2, Q,
                CV_CALIB_ZERO_DISPARITY, new_size, 0.0, 1.1);

        cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, new_size, CV_16SC2, rect_map_[0][0], rect_map_[0][1]);
        cv::fisheye::initUndistortRectifyMap(K2, D2, R2, P2, new_size, CV_16SC2, rect_map_[1][0], rect_map_[1][1]);
    }

    inline void math_flann_surf(const cv::Mat &img_1, const cv::Mat &img_2) {

        if( !img_1.data || !img_2.data ) {
            std::cout << " --(!) Error reading images " << std::endl;
            return;
        }

        //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
        int minHessian = 400;
        Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
        detector->setHessianThreshold(minHessian);

        std::vector<KeyPoint> keypoints_1, keypoints_2;
        cv::Mat descriptors_1, descriptors_2;
        detector->detectAndCompute( img_1, Mat(), keypoints_1, descriptors_1 );
        detector->detectAndCompute( img_2, Mat(), keypoints_2, descriptors_2 );

        //-- Step 2: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
        matcher.match( descriptors_1, descriptors_2, matches );

        //-- Quick calculation of max and min distances between keypoints
        double max_dist = 0; double min_dist = 100;
        for( int i = 0; i < descriptors_1.rows; i++ ) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );

        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
        //-- small)
        //-- PS.- radiusMatch can also be used here.
        std::vector< DMatch > good_matches;
        for( int i = 0; i < descriptors_1.rows; i++ ) {
            if (matches[i].distance <= max(2 * min_dist, 0.02))
                good_matches.push_back(matches[i]);
        }

        //-- Draw only "good" matches
        Mat img_matches;
        drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        cv::imshow( "FLANN Good Matches (SURF)", img_matches );
        double mean_error = 0.0;
        for( int i = 0; i < (int)good_matches.size(); i++ ) {
            int idx1 = good_matches[i].queryIdx;
            int idx2 = good_matches[i].trainIdx;
            cv::Point2f pt1 = keypoints_1[idx1].pt;
            cv::Point2f pt2 = keypoints_2[idx2].pt;
            std::cout << "\"-- Good Match [" << std::setw(2) << i << "] Keypoint 1: "
                      << std::setw(4) << idx1 << "  -- Keypoint 2: " << std::setw(4) << idx2 << " --> "
                      << pt1 << " <--> " << pt2 << std::endl;
            mean_error += std::abs(pt1.y-pt2.y);
        }
        mean_error /= good_matches.size();

        std::cout << "-- Mean Error (y): " << mean_error << std::endl;
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

    cv::Mat rect_map_[2][2];
};

#endif //CAPTURE_CAM_CALIB_FISHEYE_STEREO_H
