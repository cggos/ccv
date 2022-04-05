//
// Created by cg on 9/26/19.
//

#include "cv/types.h"

namespace cg {

    Point2f operator*(int n, const Point2f &pt) {
        return pt * n;
    }
}