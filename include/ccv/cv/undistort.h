//
// Created by cg on 10/17/19.
//

#ifndef CCV_CV_UNDISTORT_H
#define CCV_CV_UNDISTORT_H

#include "ccv/common/types.h"
#include "ccv/maths/mat.h"
#include "ccv/maths/vector.h"

namespace cg {

/**
 * @brief same with cv::undistortPoints, Computes the ideal point coordinates from the observed point coordinates
 * @param pts_in
 * @param pts_out
 * @param camera_matrix
 * @param distortion_coeffs
 * @param R
 * @param P
 */
void undistort_points(const std::vector<cg::Point2f> &pts_in,
                      std::vector<cg::Point2f> &pts_out,
                      const cg::Mat3 &camera_matrix,
                      const cg::Vector4 &distortion_coeffs,
                      const cg::Mat3 &R = cg::Matrix::eye(3),
                      const cg::Mat3 &P = cg::Matrix::eye(3));

/**
 * @brief same with cv::fisheye::undistortPoints, UnDistorts 2D points using fisheye model
 * @param pts_in
 * @param pts_out
 * @param camera_matrix
 * @param distortion_coeffs
 * @param R
 * @param P
 */
void undistort_points_fisheye(const std::vector<cg::Point2f> &pts_in,
                              std::vector<cg::Point2f> &pts_out,
                              const cg::Mat3 &camera_matrix,
                              const cg::Vector4 &distortion_coeffs,
                              const cg::Mat3 &R = cg::Matrix::eye(3),
                              const cg::Mat3 &P = cg::Matrix::eye(3));

/**
 * @brief same with cv::fisheye::distortPoints, Distorts 2D points using fisheye model
 * @param pts_in
 * @param pts_out
 * @param camera_matrix
 * @param distortion_coeffs
 */
void distort_points_fisheye(const std::vector<cg::Point2f> &pts_in,
                            std::vector<cg::Point2f> &pts_out,
                            const cg::Mat3 &camera_matrix,
                            const cg::Vector4 &distortion_coeffs);
}  // namespace cg

#endif  // CCV_CV_UNDISTORT_H
