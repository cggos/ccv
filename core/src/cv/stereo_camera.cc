#include "cvkit/cv/stereo_camera.h"

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>

namespace cg {

    void StereoCamera::compute_disparity_map(const cv::Mat &mat_l, const cv::Mat &mat_r, cv::Mat &mat_disp) {

        if (mat_l.empty() || mat_r.empty()) {
            std::cerr << "[cvkit] " << __FUNCTION__ << " : mat_l or mat_r is empty !" << std::endl;
            return;
        }
        if (mat_l.channels() != 1 || mat_r.channels() != 1) {
            std::cerr << "[cvkit] " << __FUNCTION__ << " : mat_l or mat_r is NOT single-channel image !" << std::endl;
            return;
        }

        cv::Mat mat_d_bm;
        {
            int blockSize_ = 15;  //15
            int minDisparity_ = 0;   //0
            int numDisparities_ = 64;  //64
            int preFilterSize_ = 9;   //9
            int preFilterCap_ = 31;  //31
            int uniquenessRatio_ = 15;  //15
            int textureThreshold_ = 10;  //10
            int speckleWindowSize_ = 100; //100
            int speckleRange_ = 4;   //4

            cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
            stereo->setBlockSize(blockSize_);
            stereo->setMinDisparity(minDisparity_);
            stereo->setNumDisparities(numDisparities_);
            stereo->setPreFilterSize(preFilterSize_);
            stereo->setPreFilterCap(preFilterCap_);
            stereo->setUniquenessRatio(uniquenessRatio_);
            stereo->setTextureThreshold(textureThreshold_);
            stereo->setSpeckleWindowSize(speckleWindowSize_);
            stereo->setSpeckleRange(speckleRange_);
            stereo->compute(mat_l, mat_r, mat_d_bm);
        }

        // stereoBM:
        // When disptype == CV_16S, the map is a 16-bit signed single-channel image,
        // containing disparity values scaled by 16
        mat_d_bm.convertTo(mat_disp, CV_32FC1, 1/16.f);
    }

    void StereoCamera::disparity_to_depth_map(const cv::Mat &mat_disp, cv::Mat &mat_depth) {

        if(!(mat_depth.type() == CV_16UC1 || mat_depth.type() == CV_32FC1))
            return;

        double baseline = camera_model_.baseline;
        double left_cx  = camera_model_.left.cx;
        double left_fx  = camera_model_.left.fx;
        double right_cx = camera_model_.right.cx;
        double right_fx = camera_model_.right.fx;

        mat_depth = cv::Mat::zeros(mat_disp.size(), mat_depth.type());

        for (int h = 0; h < (int) mat_depth.rows; h++) {
            for (int w = 0; w < (int) mat_depth.cols; w++) {

                float disp = 0.f;

                switch (mat_disp.type()) {
                    case CV_16SC1:
                        disp = mat_disp.at<short>(h, w);
                        break;
                    case CV_32FC1:
                        disp = mat_disp.at<float>(h, w);
                        break;
                    case CV_8UC1:
                        disp = mat_disp.at<unsigned char>(h, w);
                        break;
                }

                float depth = 0.f;
                if (disp > 0.0f && baseline > 0.0f && left_fx > 0.0f) {
                    //Z = baseline * f / (d + cx1-cx0);
                    double c = 0.0f;
                    if (right_cx > 0.0f && left_cx > 0.0f)
                        c = right_cx - left_cx;
                    depth = float(left_fx * baseline / (disp + c));
                }

                switch(mat_depth.type()) {
                    case CV_16UC1:
                        {
                            unsigned short depthMM = 0;
                            if (depth <= (float) USHRT_MAX)
                                depthMM = (unsigned short) depth;
                            mat_depth.at<unsigned short>(h, w) = depthMM;
                        }
                        break;
                    case CV_32FC1:
                        mat_depth.at<float>(h, w) = depth;
                        break;
                }
            }
        }
    }

};
