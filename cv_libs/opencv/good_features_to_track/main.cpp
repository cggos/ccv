#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "good_features_to_track.h"

#include "tic_toc.h"

int main() {

    // cv::Mat img = cv::imread("/home/cg/Pictures/jingdong.png", 0);
    cv::Mat img = cv::imread("../../../data/lena.bmp", 0);

    int num = 200;
    int min_distance = 20;

    TicToc t_feat01;
    std::vector<cv::Point2f> corners01;
    cv::goodFeaturesToTrack(img, corners01, num, 0.01, min_distance);
    printf("t_feat01: %f ms\n", t_feat01.toc());

    TicToc t_feat02;
    std::vector<cv::Point2f> corners02;
    good_features_to_track(img, corners02, num, 0.01, min_distance);
    printf("t_feat02: %f ms\n", t_feat02.toc());

    cv::Mat img_show;
    cv::cvtColor(img, img_show, cv::COLOR_GRAY2BGR);
    for (const auto& pt : corners01)
        cv::circle(img_show, pt, 3, cv::Scalar(0, 0, 255), 5);
    for (const auto& pt : corners02)
        cv::circle(img_show, pt, 2, cv::Scalar(0, 255, 0), 2);

    cv::imshow("show", img_show);
    cv::waitKey(0);

    return 0;
}