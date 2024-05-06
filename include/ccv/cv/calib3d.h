//
// Created by cg on 10/17/19.
//

#ifndef CCV_CV_CALIB3D_H
#define CCV_CV_CALIB3D_H

#include <vector>

#include "ccv/common/types.h"
#include "ccv/maths/mat.h"
#include "ccv/maths/vector.h"

namespace cg {

/**
 * @brief cv::convertPointsToHomogeneous + cv::projectPoints
 * @param pts_in
 * @param pts_out
 * @param rvec
 * @param tvec
 * @param camera_matrix
 * @param distortion_coeffs
 */
void project_points(const std::vector<cg::Point2f> &pts_in,
                    std::vector<cg::Point2f> &pts_out,
                    const cg::Vector3 &rvec,
                    const cg::Vector3 &tvec,
                    const cg::Mat3 &camera_matrix,
                    const cg::Vector4 &distortion_coeffs);

}  // namespace cg

#endif  // CCV_CV_CALIB3D_H
