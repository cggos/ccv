//
// Created by cg on 9/20/19.
//

#include "ccv/kinematics/quarternion.h"

namespace cg {

    Quarternion::Quarternion() {
#if Q_HAMILTON
        v4_[0] = 1.0;
        v4_[1] = 0.0;
        v4_[2] = 0.0;
        v4_[3] = 0.0;
#else
        v4_[0] = 0.0;
        v4_[1] = 0.0;
        v4_[2] = 0.0;
        v4_[3] = 1.0;
#endif
    }

    Quarternion::Quarternion(FLOAT w, FLOAT x, FLOAT y, FLOAT z) {
#if Q_HAMILTON
        v4_[0] = w;
        v4_[1] = x;
        v4_[2] = y;
        v4_[3] = z;
#else
        v4_[0] = x;
        v4_[1] = y;
        v4_[2] = z;
        v4_[3] = w;
#endif
    }

    Quarternion::Quarternion(const Vector<4> &v4) {
        assert(v4.size() == 4);
        v4_ = v4;
    }

    Quarternion Quarternion::unit_random() {
        srand((unsigned)time(0));
        double u1 = rand() / double(RAND_MAX); // [0, 1]
        double u2 = rand() / double(RAND_MAX) * M_2_PI;
        double u3 = rand() / double(RAND_MAX) * M_2_PI;
        double a = std::sqrt(1 - u1);
        double b = std::sqrt(u1);
        // TODO: Hamilton ?
        return Quarternion(a*sin(u2), a*cos(u2), b*sin(u3), b*cos(u3)).normalized();
    }

    Quarternion Quarternion::small_angle_quaternion(const Vector<3> &v3) {
        Vector<3> dq = v3 / 2.0;
        Quarternion q;
        double dq_square_norm = std::pow(dq.l2norm(), 2);
        if (dq_square_norm <= 1) {
            q.set_vec(dq);
            q.w() = std::sqrt(1 - dq_square_norm);
        } else {
            q.set_vec(dq);
            q.w() = 1;
            q = q / std::sqrt(1 + dq_square_norm);
        }
        return q;
    }

    Quarternion Quarternion::small_angle_quaternion(const VectorX &v) {
        assert(v.size() == 3);
        Vector3 v3;
        v3[0] = v[0];
        v3[1] = v[1];
        v3[2] = v[2];
        return small_angle_quaternion(v3);
    }

    const Matrix Quarternion::rotation_matrix() const {
        Matrix R(3, 3);
#if Q_HAMILTON
        // TODO
#else
        R = (2 * w() * w() - 1) * Matrix::eye(3) - 2 * w() * skew_symmetric(vec()) + 2 * vec() * vec().transpose();
        //TODO: Is it necessary to use the approximation equation (Equation (87)) when the rotation angle is small?
#endif
        return R;
    }

    const AngleAxis Quarternion::angle_axis() const {
        FLOAT angle;
        Vector3 v3;

//        angle = 2 * std::acos(w());
//        v3  = vec() / std::sin(angle * 0.5);

        FLOAT norm = vec().l2norm();
        angle = 2 * std::atan2(norm, w());
        v3 = vec() / norm;

//        // to be same with Eigen::AngleAxisd
//        if(angle < M_PI) {
//            angle = 2 * M_PI - angle;
//        }
        return AngleAxis(angle, v3);
    }

    Matrix Quarternion::left_product_matrix() const {
        Matrix L(4,4);
#if Q_HAMILTON
        // TODO
#else
        L(0, 0) =  v4_[3]; L(0, 1) =  v4_[2]; L(0, 2) = -v4_[1]; L(0, 3) =  v4_[0];
        L(1, 0) = -v4_[2]; L(1, 1) =  v4_[3]; L(1, 2) =  v4_[0]; L(1, 3) =  v4_[1];
        L(2, 0) =  v4_[1]; L(2, 1) = -v4_[0]; L(2, 2) =  v4_[3]; L(2, 3) =  v4_[2];
        L(3, 0) = -v4_[0]; L(3, 1) = -v4_[1]; L(3, 2) = -v4_[2]; L(3, 3) =  v4_[3];
#endif
        return L;
    }
}

