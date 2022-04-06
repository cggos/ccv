//
// Created by cg on 9/27/19.
//

#ifndef MSCKF_ANGLE_AXIS_H
#define MSCKF_ANGLE_AXIS_H

#include "cvkit/maths/vector.h"

namespace cg {

    class AngleAxis {
    public:
        AngleAxis() {}

        AngleAxis(const FLOAT &angle, const Vector3 &axis) : angle_(angle), axis_(axis) {}

        const FLOAT angle() const { return angle_; }

        FLOAT &angle() { return angle_; }

        const Vector3 axis() const { return axis_; }

        Vector3 &axis() { return axis_; }

        friend std::ostream &operator<<(std::ostream &os, const AngleAxis &angle_axis) {
            os << angle_axis.angle_ << ", "
               << angle_axis.axis_[0] << " "
               << angle_axis.axis_[1] << " "
               << angle_axis.axis_[2];
            return os;
        }

    private:
        FLOAT angle_;
        Vector3 axis_;
    };
}

#endif //MSCKF_ANGLE_AXIS_H
