//
// Created by cg on 9/16/19.
//

#include <fstream>
#include <chrono>

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "cvkit/common/types.h"
#include "cvkit/cv/yimg.h"
#include "cvkit/cv/corner_detector.h"
#include "cvkit/cv/image_filtering.h"
#include "cvkit/cv/visual_tracking.h"

TEST(cv, Size)
{
    cg::Size sz = cg::Size();
    std::cout << "sz: " << sz << std::endl;

    cg::Size sza = cg::Size(100,200);
    cg::Size szb = cg::Size(200,300);
    cg::Size szc = sza + szb;
    std::cout << "szc: " << szc << ", area: " << szc.area() << std::endl;
}

TEST(cv, Point2D)
{
    cg::Point2i p;
    std::cout<< "p: " << p <<std::endl;
   
    const int n=2; 
    cg::Point2f a(100,100);
    cg::Point2f b(200,200);
    cg::Point2f c = a * n;
    c = 3 * a;
    c = -c;
    std::cout<<a<<"*"<<n<<" = "<<c<<std::endl;

    cg::Point2f d = a + b;
    std::cout<<a<<"+"<<b<<" = "<<d<<std::endl;

    cv::Vec3d pt1(1,1,1);
    cv::Vec3d pt2(2,2,2);
    double val = (pt1.t() * pt2)[0];
    std::cout << "pt: " << pt1.t() * pt2 << std::endl;
}

TEST(YImg, copy)
{
    cv::Mat mat_src = cv::imread("../../data/lena.bmp", cv::ImreadModes::IMREAD_GRAYSCALE);

    ASSERT_FALSE(mat_src.empty());

    cg::YImg8 yimg_src(mat_src.rows, mat_src.cols);
    memcpy(yimg_src.data(), mat_src.data, yimg_src.size().area());

    cg::YImg8 yimg_dst(yimg_src.size());
    yimg_src.copy(yimg_dst);
    // yimg_dst = yimg_src;

    cv::Mat mat_dst;
    mat_dst.create(mat_src.rows, mat_src.cols, CV_8UC1);
    mempcpy(mat_dst.data, yimg_dst.data(), yimg_dst.size().area());

    cv::imshow("YImg copy test", mat_dst);
    cv::waitKey(1000);
}

TEST(CornerDetector, detect_features)
{
    cv::Mat mat_src = cv::imread("../../data/lena.bmp", cv::ImreadModes::IMREAD_GRAYSCALE);

    cg::YImg8 yimg_src(mat_src.rows, mat_src.cols);
    memcpy(yimg_src.data(), mat_src.data, yimg_src.size().area());

    std::vector<cg::Point2f> new_features;
    std::vector<double> nm_scores;
    cg::CornerDetector detector;
    detector.detect_features(yimg_src, new_features, nm_scores);

    cv::Mat mat_dst(mat_src.rows, mat_src.cols, CV_8UC3);
    cv::cvtColor(mat_src, mat_dst, CV_GRAY2BGR);
    for (const auto &pt : new_features) {
        cv::circle(mat_dst, cv::Point2f(pt.x, pt.y), 3, cv::Scalar(0, 255, 0), -1);
    }
    cv::imshow("CornerDetector FAST", mat_dst);
    cv::waitKey(1000);
}

TEST(ImageFiltering, gaussian_blur) {
    cv::Mat mat_src = cv::imread("../../data/lena.bmp", cv::ImreadModes::IMREAD_GRAYSCALE);

    ASSERT_FALSE(mat_src.empty());

    cg::YImg8 yimg_src(mat_src.rows, mat_src.cols);
    memcpy(yimg_src.data(), mat_src.data, yimg_src.size().area());

    cg::YImg8 yimg_dst(yimg_src.size());
    cg::gaussian_blur(yimg_src, yimg_dst, 5, 0.84089642);

    cv::Mat mat_dst_01;
    mat_dst_01.create(mat_src.rows, mat_src.cols, CV_8UC1);
    mempcpy(mat_dst_01.data, yimg_dst.data(), yimg_dst.size().area());

    cv::Mat mat_dst_02;
    cv::GaussianBlur(mat_src, mat_dst_02, cv::Size(5, 5), 0.84089642);

    cg::pyr_down(yimg_src, yimg_dst);
    cv::Mat mat_dst_03(yimg_dst.rows(), yimg_dst.cols(), CV_8UC1);
    mempcpy(mat_dst_03.data, yimg_dst.data(), yimg_dst.size().area());

    cv::imshow("GaussianBlur YImg", mat_dst_01);
    cv::imshow("GaussianBlur OCV", mat_dst_02);
    cv::imshow("PyrDown YImg", mat_dst_03);
    cv::waitKey(1000);
}

TEST(VisualTracking, optical_flow) {
    cv::Mat mat_src_01 = cv::imread("../../data/optical_flow_01.png", cv::ImreadModes::IMREAD_GRAYSCALE);
    cv::Mat mat_src_02 = cv::imread("../../data/optical_flow_02.png", cv::ImreadModes::IMREAD_GRAYSCALE);

    cg::YImg8 yimg_src_01(mat_src_01.rows, mat_src_01.cols);
    memcpy(yimg_src_01.data(), mat_src_01.data, yimg_src_01.size().area());
    cg::YImg8 yimg_src_02(mat_src_02.rows, mat_src_02.cols);
    memcpy(yimg_src_02.data(), mat_src_02.data, yimg_src_02.size().area());

    /// detect Features
    std::vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(mat_src_01, kp1);

    std::vector<cv::Point2f> cv_pt1;
    std::vector<cg::Point2f> cg_pt1;
    for (auto &kp: kp1) {
        cv_pt1.push_back(kp.pt);
        cg_pt1.push_back(cg::Point2f(kp.pt.x, kp.pt.y));
    }

    /// use opencv's multi-level pyramids lk optical flow cv::calcOpticalFlowPyrLK
    std::vector<cv::Mat> img1_pyramid, img2_pyramid;
    cv::buildOpticalFlowPyramid(mat_src_01, img1_pyramid, cv::Size(15, 15), 3, true, cv::BORDER_REFLECT_101, cv::BORDER_CONSTANT, false);
    cv::buildOpticalFlowPyramid(mat_src_02, img2_pyramid, cv::Size(15, 15), 3, true, cv::BORDER_REFLECT_101, cv::BORDER_CONSTANT, false);

    auto start_time = std::chrono::steady_clock::now();
    std::vector<cv::Point2f> cv_pt2_multi;
    std::vector<uchar> status_multi;
    cv::calcOpticalFlowPyrLK(
            img1_pyramid, img2_pyramid, cv_pt1, cv_pt2_multi, status_multi, cv::noArray(), cv::Size(15, 15), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
    auto end_time = std::chrono::steady_clock::now();
    std::cout << "time calcOpticalFlowPyrLK: " << std::chrono::duration<double>(end_time-start_time).count() << " s" << std::endl;

    /// cg::OpticalFlowMultiLevel
    std::vector<cg::YImg8> pyr1, pyr2; // image pyramids
    cg::YImg8 tmp1, tmp2;
    for (int i = 0; i < 4; i++) {
        if(i == 0) {
            pyr1.push_back(yimg_src_01);
            pyr2.push_back(yimg_src_02);
            continue;
        }
        cg::pyr_down(pyr1[i-1], tmp1);
        cg::pyr_down(pyr2[i-1], tmp2);
        pyr1.push_back(tmp1);
        pyr2.push_back(tmp2);
    }
    start_time = std::chrono::steady_clock::now();
    std::vector<cg::Point2f> kp2_multi;
    std::vector<unsigned char> success_multi;
    cg::optical_flow_multi_level(pyr1, pyr2, cg_pt1, kp2_multi, success_multi, 15, 30);
    end_time = std::chrono::steady_clock::now();
    std::cout << "time optical_flow_multi_level: " << std::chrono::duration<double>(end_time-start_time).count() << " s" << std::endl;

    /// write
    std::ofstream out_file("debug_optical_flow.txt");
    out_file << "OpticalFlowMultiLevel <--> calcOpticalFlowPyrLK" << std::endl;
    for(int i=0; i<status_multi.size();  ++i) {
        out_file << kp2_multi[i] << ", " << (int)success_multi[i] << " <--> "
                 << cv_pt2_multi[i] << ", " << (int)status_multi[i] << std::endl;
    }
    out_file.close();

    /// draw
    cv::Mat cv_img2_multi;
    cv::cvtColor(mat_src_02, cv_img2_multi, CV_GRAY2BGR);
    for (int i = 0; i < cv_pt2_multi.size(); i++) {
        if (status_multi[i]) {
            cv::circle(cv_img2_multi, cv_pt2_multi[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(cv_img2_multi, cv_pt1[i], cv_pt2_multi[i], cv::Scalar(0, 250, 0));
        }
    }

    cv::Mat img2_multi;
    cv::cvtColor(mat_src_02, img2_multi, CV_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++) {
        if (success_multi[i]) {
            cv::circle(img2_multi, cv::Point2f(kp2_multi[i].x, kp2_multi[i].y), 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, cv::Point2f(kp2_multi[i].x, kp2_multi[i].y), cv::Scalar(0, 250, 0));
        }
    }

    cv::imshow("tracked by opencv lk multi", cv_img2_multi);
    cv::imshow("tracked multi level", img2_multi);
    while (cv::waitKey(0) != 27);
}