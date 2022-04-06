//
// Created by cg on 9/16/19.
//

#ifndef VIKIT_CG_QUARTERNION_H
#define VIKIT_CG_QUARTERNION_H

#include <assert.h>
#include <cmath>

#include "cvkit/kinematics/angle_axis.h"

namespace cg {

#define Q_HAMILTON 0 // 0 for JPL, 1 for HAMILTON

    /**
     * @brief Hamilton and JPL Quarternion
     */
    class Quarternion {
    public:
        Quarternion();

        Quarternion(FLOAT w, FLOAT x, FLOAT y, FLOAT z);

        Quarternion(const Vector<4> &v4);

        const FLOAT w() const {
#if Q_HAMILTON
            return v4_[0];
#else
            return v4_[3];
#endif
        }

        FLOAT &w() {
#if Q_HAMILTON
            return v4_[0];
#else
            return v4_[3];
#endif
        }

        const FLOAT x() const {
#if Q_HAMILTON
            return v4_[1];
#else
            return v4_[0];
#endif
        }

        FLOAT &x() {
#if Q_HAMILTON
            return v4_[1];
#else
            return v4_[0];
#endif
        }

        const FLOAT y() const {
#if Q_HAMILTON
            return v4_[2];
#else
            return v4_[1];
#endif
        }

        FLOAT &y() {
#if Q_HAMILTON
            return v4_[2];
#else
            return v4_[1];
#endif
        }

        const FLOAT z() const {
#if Q_HAMILTON
            return v4_[3];
#else
            return v4_[2];
#endif
        }

        FLOAT &z() {
#if Q_HAMILTON
            return v4_[3];
#else
            return v4_[2];
#endif
        }

        const Vector<3> vec() const {
#if Q_HAMILTON
            return v4_.block<3>(1);
#else
            return v4_.block<3>(0);
#endif
        }

        inline void set_vec(const Vector<3> &v3) {
            assert(v3.size() == 3);
            x() = v3[0];
            y() = v3[1];
            z() = v3[2];
        }

        const Vector<4> vector4() const { return v4_; }

        /**
         * @brief Normalize the given quaternion to unit quaternion
         */
        inline Quarternion normalized() {
            FLOAT l2norm = v4_.l2norm();
            return this->v4_ / l2norm;
        }

        /**
         * @brief unit random quarternion, ref: http://planning.cs.uiuc.edu/node198.html
         * @return
         */
        static Quarternion unit_random();

        /**
         * @brief Convert the vector part of a quaternion to a full quaternion.
         * @note This function is useful to convert delta quaternion which is usually a 3x1 vector to a full quaternion.
         *       For more details, check Section 3.2 "Kalman Filter Update" in
         *       "Indirect Kalman Filter for 3D Attitude Estimation: A Tutorial for quaternion Algebra".
         * @param v3
         * @return
         */
        static Quarternion small_angle_quaternion(const Vector<3> &v3);

        static Quarternion small_angle_quaternion(const VectorX &v3);

        /**
         * @brief Convert a quaternion to the corresponding rotation matrix
         * @note Pay attention to the convention used. The function follows the conversion in
         *      "Indirect Kalman Filter for 3D Attitude Estimation: A Tutorial for Quaternion Algebra", Equation (78).
         *
         *       The input quaternion should be in the form [q1, q2, q3, q4(scalar)]^T
         * @return
         */
        const Matrix rotation_matrix() const;

        /**
         * @brief Quarternion to Angle Axis
         *        ref: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/
         * @return
         */
        const AngleAxis angle_axis() const; // TODO: verify

        Matrix left_product_matrix() const;

        FLOAT &operator[](int n) { return v4_[n]; }

        const Quarternion &operator-() {
            v4_ = -v4_;
            return (*this);
        }

        Quarternion operator*(const Quarternion &q) const {
//            Vector<4> v4 = cg::template operator*<4,4>(this->left_product_matrix(), q.v4_);
            Vector<4> v4 = this->left_product_matrix() * q.v4_;
            return Quarternion(v4).normalized();
        }

        Quarternion operator/(const FLOAT &s) {
            Quarternion q;
            q.v4_ = this->v4_ / s;
            return q;
        }

        friend std::ostream &operator<<(std::ostream &out, Quarternion q) {
#if Q_HAMILTON
            out << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
#else
            out << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
#endif
            return out;
        }

    private:
        Vector<4> v4_;
    };
}

#endif //VIKIT_CG_QUARTERNION_H
