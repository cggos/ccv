#include "cgocv/stereo_camera.h"

#include <opencv2/calib3d/calib3d.hpp>

namespace cg {

    void StereoCamera::compute_depth_map(const cv::Mat &mat_left, const cv::Mat &mat_right,
            cv::Mat &mat_depth) {

        if(mat_left.empty() || mat_right.empty())
            return;

        if(!(mat_depth.type() == CV_16UC1 || mat_depth.type() == CV_32FC1))
            return;

        double baseline = camera_model_.baseline;
        double left_cx  = camera_model_.left.cx;
        double left_fx  = camera_model_.left.fx;
        double right_cx = camera_model_.right.cx;
        double right_fx = camera_model_.right.fx;

        cv::Mat mat_disp;
        {
            int blockSize_         = 15;  //15
            int minDisparity_      = 0;   //0
            int numDisparities_    = 64;  //64
            int preFilterSize_     = 9;   //9
            int preFilterCap_      = 31;  //31
            int uniquenessRatio_   = 15;  //15
            int textureThreshold_  = 10;  //10
            int speckleWindowSize_ = 100; //100
            int speckleRange_      = 4;   //4

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
            stereo->compute(mat_left, mat_right, mat_disp);
        }

        mat_depth = cv::Mat::zeros(mat_left.size(), mat_depth.type());

        for (int h = 0; h < (int) mat_depth.rows; h++) {
            for (int w = 0; w < (int) mat_depth.cols; w++) {

                float disp = 0.f;

                switch (mat_disp.type()) {
                    case CV_16SC1:
                        disp = float(mat_disp.at<short>(h, w)) / 16.0f;
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
                } else
                    depth = std::numeric_limits<float>::quiet_NaN();

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

    void StereoCamera::depth_to_pointcloud(
            const cv::Mat &mat_depth, const cv::Mat &mat_left,
            pcl::PointCloud<pcl::PointXYZRGB> &point_cloud) {

        point_cloud.height = (uint32_t) mat_depth.rows;
        point_cloud.width  = (uint32_t) mat_depth.cols;
        point_cloud.is_dense = false;
        point_cloud.resize(point_cloud.height * point_cloud.width);

        for (int h = 0; h < (int) mat_depth.rows; h++) {
            for (int w = 0; w < (int) mat_depth.cols; w++) {

                pcl::PointXYZRGB &pt = point_cloud.at(h * point_cloud.width + w);

                unsigned char v = mat_left.at<unsigned char>(h, w);
                pt.b = v;
                pt.g = v;
                pt.r = v;

                float depth = 0.f;
                switch (mat_depth.type()) {
                    case CV_16UC1: // unit is mm
                        depth = float(mat_depth.at<unsigned short>(h,w));
                        depth *= 0.001f;
                        break;
                    case CV_32FC1: // unit is meter
                        depth = mat_depth.at<float>(h,w);
                        break;
                }

                double W = depth / camera_model_.left.fx;
                if (std::isfinite(depth) && depth >= 0) {
                    pt.x = float((cv::Point2f(w, h).x - camera_model_.left.cx) * W);
                    pt.y = float((cv::Point2f(w, h).y - camera_model_.left.cy) * W);
                    pt.z = depth;
                } else
                    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

};
