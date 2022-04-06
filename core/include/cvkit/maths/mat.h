//
// Created by cg on 9/27/19.
//

#ifndef MSCKF_MAT_H
#define MSCKF_MAT_H

#include "cvkit/maths/matrix.h"

namespace cg {

    template<unsigned int _M, unsigned int _N>
    class Mat : public Matrix {
    public:
        Mat() : Matrix(_M, _N) {}

        Mat(const Matrix &mat) : Matrix(mat) {}
    };

    typedef Mat<2, 2> Mat2;
    typedef Mat<3, 3> Mat3;
    typedef Mat<4, 4> Mat4;
}

#endif //MSCKF_MAT_H
