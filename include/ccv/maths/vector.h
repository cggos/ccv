//
// Created by cg on 9/17/19.
//

#ifndef VIKIT_CG_VECTOR_H
#define VIKIT_CG_VECTOR_H

#include "matrix.h"

#include <cmath>
#include <assert.h>
#include <initializer_list>

namespace cg {

    /**
     * @brief column vector
     */
    class VectorX : public Matrix {
    public:
        VectorX() : Matrix() {}

        VectorX(int n) : Matrix(n, 1) {}

        VectorX(const Matrix &mat) : Matrix(mat) { assert(mat.cols() == 1); }

        FLOAT &operator[](int idx) { return (*this)(idx, 0); }

        const FLOAT &operator[](int idx) const { return (*this)(idx, 0); }

        inline unsigned int size() const { return rows(); }

        void conservative_resize(int32_t m) {
            Matrix m_tmp = *this;
            m_tmp.conservative_resize(m, 1);
            *this = m_tmp;
        }

        template<unsigned int _M>
        VectorX head() const { return this->get_mat(0, 0, _M-1, 0); }

        template<unsigned int _M>
        VectorX tail() const { return this->get_mat(rows()-_M, 0, rows()-1, 0); }

        template<unsigned int _M>
        inline VectorX segment(int idx) const { return this->get_mat(idx, 0, idx+_M-1, 0); }

        void set_segment(int idx, const VectorX &v) { this->set_mat(idx, 0, v); }
    };

    /**
     * @brief column vector
     */
    template<unsigned int _N>
    class Vector : public Matrix {
    public:
        Vector() : Matrix(_N, 1) {}

        Vector(std::initializer_list<FLOAT> vlist) : Matrix(_N, 1) {
            int i = 0;
            for (auto a : vlist) {
                if (i == _N)
                    break;
                (*this)(i, 0) = a;
                ++i;
            }
            for (; i < _N; ++i)
                (*this)(i, 0) = 0.0;
        }

        Vector(const FLOAT *val) : Matrix(_N, 1, val) {}

        Vector(const Matrix &mat) : Matrix(mat) { assert(mat.rows() ==_N && mat.cols() == 1); }

        FLOAT &operator[](int idx) { return (*this)(idx, 0); }

        const FLOAT &operator[](int idx) const { return (*this)(idx, 0); }

        inline unsigned int size() const { return _N; }

        Vector operator*(const FLOAT &m) const { return Matrix(*this) * m; }

        Matrix operator*(const Vector &v) const { return Matrix(*this) * Matrix(v); }

        Vector operator/(const FLOAT &m) const { return Matrix(*this) / m; }

        void operator+=(const Vector &v) { *this = Matrix(*this) + Matrix(v); }

        void operator*=(const FLOAT &m) { *this = Matrix(*this) * m; }

        inline FLOAT l1norm() const {
            FLOAT norm = 0;
            for(int i=0; i<_N; ++i)
                norm += (*this)(i, 0);
            return norm;
        }

        inline FLOAT squared_l2norm() const { return std::pow(l2norm(), 2); }

        inline FLOAT dot(const Vector &v) {
            assert(_N == v.size());
            FLOAT sum = 0.0;
            for(int i=0; i<v.size(); ++i)
                sum += (*this)[i] * v[i];
            return sum;
        }

        template<unsigned int _M>
        inline Vector<_M> block(int idx) const { return this->get_mat(idx, 0, idx+_M-1, 0); }

        inline FLOAT max_coeff(int &i_max) {
            FLOAT max = 0.0;
            i_max = 0;
            for(int i=0; i<size(); ++i) {
                if((*this)[i]>max) {
                    max = (*this)[i];
                    i_max = i;
                }
            }
            return max;
        }
    };

    typedef Vector<2> Vector2;
    typedef Vector<3> Vector3;
    typedef Vector<4> Vector4;
    typedef Vector<5> Vector5;
    typedef Vector<6> Vector6;

    /**
     * @brief Create a skew-symmetric matrix from a 3-element vector.
     * @param w
     * @note Performs the operation:
     *  w   ->  [  0 -w3  w2]
     *          [ w3   0 -w1]
     *          [-w2  w1   0]
     * @return
     */
    Matrix skew_symmetric(const Vector<3> &w);

    /**
     * @brief s * M
     * @param s
     * @param M
     * @return
     */
    Matrix operator*(FLOAT s, Matrix M);

    /**
     * @brief s * V
     * @param s
     * @param V
     * @return
     */
    template<unsigned int _M>
    Vector<_M> operator*(FLOAT s, Vector<_M> V)  { return Matrix(V) * s; }

    /**
     * @brief M * V
     * @param M
     * @param V
     * @return
     */
    template<unsigned int _M, unsigned int _N>
    static Vector<_M> operator*(const Matrix &M, const Vector<_N> &V) {
        assert(M.cols() == V.size() && M.rows() == _M);
        Vector<_M> v;
        for(int i=0; i<M.rows(); ++i) {
            FLOAT sum_i = 0.0;
            for(int j=0; j<M.cols(); ++j)
                sum_i += M(i,j) * V[j];
            v[i] = sum_i;
        }
        return v;
    }

    /**
     * @brief column Vector to Matrix
     * @param v1
     * @param v2
     * @return
     */
    Matrix vec2mat(const Vector2 &v1, const Vector2 &v2);

    Matrix vec2mat(const VectorX &v1, const VectorX &v2);

    VectorX solve_ldlt(const Matrix &A, const VectorX &b);
}

#endif //VIKIT_CG_VECTOR_H
