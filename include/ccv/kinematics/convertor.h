//
// Created by cg on 9/27/19.
//

#ifndef MSCKF_CONVERTOR_H
#define MSCKF_CONVERTOR_H

#include "ccv/kinematics/rotation_matrix.h"

namespace cg {

    /**
     * @brief Converts a rotation vector to a rotation matrix
     *        ref: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#rodrigues
     * @param v3 rotation vector
     * @return
     */
    RotationMatrix rodrigues(const Vector3 &v3);

    /**
     * @brief Converts a rotation vector to a rotation matrix
     * @param v3_hat unit rotation vector
     * @param s sin theta
     * @param c cos theta
     * @return
     */
    RotationMatrix rodrigues(const Vector3 &v3_hat, FLOAT s, FLOAT c);

    /**
     * @brief get a rotation matrix between two arbitrary vectors
     *        ref: https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
     * @param v31
     * @param v32
     * @return
     */
    RotationMatrix from_two_vector(const Vector3 &v31, const Vector3 &v32);
}

#endif //MSCKF_CONVERTOR_H
