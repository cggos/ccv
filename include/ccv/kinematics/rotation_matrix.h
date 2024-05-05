//
// Created by cg on 9/16/19.
//

#ifndef VIKIT_CG_ROTATION_MATRIX_H
#define VIKIT_CG_ROTATION_MATRIX_H

#include "ccv/kinematics/quarternion.h"
#include "ccv/kinematics/angle_axis.h"

namespace cg {

    class RotationMatrix : public Matrix {
    public:
        RotationMatrix();

        RotationMatrix(const Matrix &mat);

        inline const FLOAT trace() const {
            FLOAT tr = 0.0;
            for(int i=0; i<3; ++i)
                tr += (*this)(i, i);
            return tr;
        }

        RotationMatrix transpose() const { return Matrix(*this).transpose(); }

        // create 3x3 rotation matrices (convention: http://en.wikipedia.org/wiki/Rotation_matrix)
        static RotationMatrix rot_mat_x(const FLOAT &angle);

        static RotationMatrix rot_mat_y(const FLOAT &angle);

        static RotationMatrix rot_mat_z(const FLOAT &angle);

        /**
         * @brief Convert a rotation matrix to a quaternion.
         * @note Pay attention to the convention used. The function follows the conversion in
         *       "Indirect Kalman Filter for 3D Attitude Estimation: A Tutorial for Quaternion Algebra", Equation (98-99).
         * @return
         */
        const Quarternion quarternion() const;

        /**
         * @brief Convert a rotation matrix to a Hamilton quaternion.
         * @return
         */
        const Quarternion quarternion_hamilton() const;

        /**
         * @brief Rotation Matrix to Axis Angle
         *        ref: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#rodrigues
         * @details
         * @return
         */
        const AngleAxis angle_axis() const; // TODO: verify
    };
}

#endif //VIKIT_CG_ROTATION_MATRIX_H
