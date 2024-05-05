//
// Created by cg on 9/25/19.
//

#ifndef MSCKF_TRANSFORM_H
#define MSCKF_TRANSFORM_H

#include "ccv/kinematics/rotation_matrix.h"

namespace cg {

    class EuclideanTransform : public Matrix {
    public:
        EuclideanTransform() : Matrix(4, 4) { update(); }

        EuclideanTransform(const Matrix &mat) : Matrix(mat) { update(false); }

        EuclideanTransform(const RotationMatrix &rm, const Vector3 &v3)
            : Matrix(4,4)
            , rotation_matrix_(rm)
            , translation_(v3) {
            update();
        }

        const RotationMatrix &rotation_matrix() const { return rotation_matrix_; }

        const Vector3 &translation() const { return translation_; }

        inline void set_rotation_matrix(const RotationMatrix &rm) {
            rotation_matrix_ = rm;
            update();
        }

        inline void set_translation(const Vector3 &v3) {
            translation_ = v3;
            update();
        }

    private:
        inline void update(bool flag=true) {
            if(flag) {
                eye();
                set_mat(0, 0, rotation_matrix_);
                set_mat(0, 3, translation_);
            } else {
                rotation_matrix_ = get_mat(0, 0, 2, 2);
                translation_ = get_mat(0, 3, 2, 3);
            }
        }

    private:
        RotationMatrix rotation_matrix_;
        Vector3 translation_;
    };
}

#endif //MSCKF_TRANSFORM_H
