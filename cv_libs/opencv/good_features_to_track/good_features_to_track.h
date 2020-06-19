#pragma once

#include <opencv2/core/core.hpp>

enum { MINEIGENVAL = 0,
       HARRIS = 1,
       EIGENVALSVECS = 2 };

struct greaterThanPtr : public std::binary_function<const float *, const float *, bool> {
    bool operator()(const float *a, const float *b) const
    // Ensure a fully deterministic result of the sort
    { return (*a > *b) ? true : (*a < *b) ? false : (a > b); }
};

void good_features_to_track(const cv::Mat &_image, std::vector<cv::Point2f> &_corners,
                            int maxCorners, double qualityLevel, double minDistance,
                            const cv::Mat &_mask = cv::Mat(), int blockSize = 3, int gradientSize = 3,
                            bool useHarrisDetector = false, double harrisK = 0.04);

void cornerMinEigenVal(const cv::Mat &_src, cv::Mat &_dst, int blockSize, int ksize = 3,
                       int borderType = cv::BORDER_DEFAULT);

void cornerEigenValsVecs(const cv::Mat &src, cv::Mat &eigenv, int block_size,
                         int aperture_size, int op_type, double k = 0.,
                         int borderType = cv::BORDER_DEFAULT);

static void calcMinEigenVal(const cv::Mat &_cov, cv::Mat &_dst);