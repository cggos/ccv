//
// Created by cg on 9/27/19.
//

#include "cvkit/kinematics/convertor.h"

namespace cg {

    RotationMatrix rodrigues(const Vector3 &v3) {
        FLOAT theta = v3.l2norm();
        if(theta == 0)
            return RotationMatrix();
        Vector3 v3_hat = v3 / theta;
        FLOAT c1 = std::cos(theta);
        FLOAT s1 = std::sin(theta);
        return rodrigues(v3_hat, s1, c1);
    }

    RotationMatrix rodrigues(const Vector3 &v3_hat, FLOAT s, FLOAT c) {
        RotationMatrix I;
        Matrix v3_skew = cg::skew_symmetric(v3_hat);
        RotationMatrix R = c * I + (1-c) * v3_hat * v3_hat.transpose() + s * v3_skew;
        return R;
    }

    RotationMatrix from_two_vector(const Vector3 &v31, const Vector3 &v32) {
        FLOAT theta1 = v31.l2norm();
        FLOAT theta2 = v32.l2norm();
        if(theta1 == 0 || theta2 == 0)
            return RotationMatrix();

        Vector3 v31_hat = v31 / theta1;
        Vector3 v32_hat = v32 / theta2;
        Vector3 v312 = Matrix::cross(v31_hat, v32_hat);
        FLOAT s = v312.l2norm();
        if(s == 0)
            return RotationMatrix();
        Vector3 v312_hat = v312 / s;
        FLOAT c = v31_hat.dot(v32_hat);

        return rodrigues(v312_hat, s, c);
    }
}
