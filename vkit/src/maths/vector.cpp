//
// Created by cg on 9/24/19.
//

#include "maths/vector.h"

namespace cg {

    /**
     * @brief Create a skew-symmetric matrix from a 3-element vector.
     * @param w
     * @note Performs the operation:
     *  w   ->  [  0 -w3  w2]
     *          [ w3   0 -w1]
     *          [-w2  w1   0]
     * @return
     */
    Matrix skew_symmetric(const Vector<3> &w) {
        Matrix w_hat(3, 3);
        w_hat(0, 0) =  0;
        w_hat(0, 1) = -w[2];
        w_hat(0, 2) =  w[1];
        w_hat(1, 0) =  w[2];
        w_hat(1, 1) =  0;
        w_hat(1, 2) = -w[0];
        w_hat(2, 0) = -w[1];
        w_hat(2, 1) =  w[0];
        w_hat(2, 2) =  0;
        return w_hat;
    }

    /**
     * @brief s * M
     * @param s
     * @param M
     * @return
     */
    Matrix operator*(FLOAT s, Matrix M) { return M * s; }

    Matrix vec2mat(const Vector2 &v1, const Vector2 &v2) {
        assert(v1.size() == v2.size());
        int m = v1.size();
        Matrix mat(m, 2);
        for(int i=0; i<m; ++i)
            mat(i, 0) = v1[i];
        for(int i=0; i<m; ++i)
            mat(i, 1) = v2[i];
        return mat;
    }

    Matrix vec2mat(const VectorX &v1, const VectorX &v2) {
        assert(v1.size() == v2.size());
        int m = v1.size();
        Matrix mat(m, 2);
        for(int i=0; i<m; ++i)
            mat(i, 0) = v1[i];
        for(int i=0; i<m; ++i)
            mat(i, 1) = v2[i];
        return mat;
    }

    VectorX solve_ldlt(const Matrix &A, const VectorX &b) {
        VectorX x(b.size());
        cg::Matrix L, D;
        A.ldlt(L, D);
        cg::Matrix Linv = L.inv();
        cg::Matrix Dinv(D.rows(), D.cols());
        for(int i=0; i<Dinv.rows(); i++)
                Dinv(i,i) = 1.0 / D(i,i);
        x = Linv.transpose() * Dinv * Linv * b;
        return x;
    }
}
