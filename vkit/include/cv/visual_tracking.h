//
// Created by cg on 10/30/19.
//

#ifndef MSCKF_VISUAL_TRACKING_H
#define MSCKF_VISUAL_TRACKING_H

#include <cmath>
#include <vector>

#include "cv/types.h"
#include "cv/yimg.h"

namespace cg {

    /**
     * single level optical flow
     * @param [in] img1 the first image
     * @param [in] img2 the second image
     * @param [in] kp1 keypoints in img1
     * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
     * @param [out] success true if a keypoint is tracked successfully
     * @param [in] inverse use inverse formulation?
     */
    void optical_flow_single_level(
            const cg::YImg8 &img1,
            const cg::YImg8 &img2,
            const std::vector <cg::Point2f> &kpt1,
            std::vector <cg::Point2f> &kpt2,
            std::vector<unsigned char> &success,
            int path_size = 7,
            int max_iters = 10,
            bool inverse = false
    );

    /**
     * multi level optical flow, scale of pyramid is set to 2 by default
     * the image pyramid will be create inside the function
     * @param [in] img1 the first pyramid
     * @param [in] img2 the second pyramid
     * @param [in] kp1 keypoints in img1
     * @param [out] kp2 keypoints in img2
     * @param [out] success true if a keypoint is tracked successfully
     * @param [in] inverse set true to enable inverse formulation
     */
    void optical_flow_multi_level(
            const std::vector<cg::YImg8> &pyr1,
            const std::vector<cg::YImg8> &pyr2,
            const std::vector<cg::Point2f> &kpt1,
            std::vector<cg::Point2f> &kpt2,
            std::vector<unsigned char> &success,
            int path_size = 7,
            int max_iters = 10,
            bool inverse = true
    );

    /**
     * get a gray scale value from reference image (bi-linear interpolated)
     * @param img
     * @param x
     * @param y
     * @return
     */
    inline float get_pixel_value(const cg::YImg8 &img, float x, float y) {
        const int lx = std::floor(x);
        const int ly = std::floor(y);
        if (lx < 0 || ly < 0 || lx+1 >= img.cols() || ly+1 >= img.rows())
            return 0.f;
        float xx = x - lx;
        float yy = y - ly;
        return float(
                (1 - xx) * (1 - yy) * img(ly, lx) +
                xx * (1 - yy) * img(ly, lx + 1) +
                (1 - xx) * yy * img(ly + 1, lx) +
                xx * yy * img(ly + 1, lx + 1)
        );
    }

}

#endif //MSCKF_VISUAL_TRACKING_H
