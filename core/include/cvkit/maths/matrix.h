#ifndef MATRIX_H
#define MATRIX_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <memory>

#include "cvkit/common.h"

namespace cg {

    class Matrix {
    public:
        Matrix();                                                    // init empty 0x0 matrix
        Matrix(const int32_t m, const int32_t n);                    // init empty mxn matrix
        Matrix(const int32_t m, const int32_t n, const FLOAT *val_); // init mxn matrix with values from array 'val'
        Matrix(const Matrix &M);                                     // creates deepcopy of M
        Matrix(const int32_t m, const int32_t n, const float *val_); // init mxn matrix with values from array 'val'

        virtual ~Matrix();

        inline const int rows() const { return m_; }
        inline const int cols() const { return n_; }

        /**
         * @brief assignment operator, copies contents of M
         * @param M
         * @return
         */
        Matrix &operator=(const Matrix &M);

        FLOAT &operator()(const int row, const int col);

        const FLOAT &operator()(const int row, const int col) const;

        // TODO
//        Matrix &block(int i1, int j1, int mm, int nn) {}

        const Matrix block(int i1, int j1, int mm, int nn) const {
            if(0 == mm || 0 == nn)
                return Matrix(mm, nn);
            Matrix mat(mm, nn);
            mat = get_mat(i1, j1, i1+mm-1, j1+nn-1);
            return mat;
        }

        // TODO
//        template<unsigned int _M, unsigned int _N>
//        Matrix &block(int i1, int j1) {}

        template<unsigned int _M, unsigned int _N>
        const Matrix block(int i1, int j1) const {
            Matrix mat(_M, _N);
            mat = get_mat(i1, j1, i1+_M-1, j1+_N-1);
            return mat;
        }

        /**
         * @brief get submatrices of current matrix
         * @param i1
         * @param j1
         * @param i2
         * @param j2
         * @return
         */
        const Matrix get_mat(int32_t i1, int32_t j1, int32_t i2 = -1, int32_t j2 = -1) const;

        /**
         * @brief set submatrices of current matrix
         * @param i
         * @param j
         * @param M
         */
        void set_mat(const int32_t i, const int32_t j, const Matrix &M);

        /**
         * @brief set (part of) diagonal to scalar, -1 as end replaces whole diagonal
         * @param s
         * @param i1
         * @param i2
         */
        void set_diag(FLOAT s, int32_t i1 = 0, int32_t i2 = -1);

        /**
         * @brief create diagonal matrix with nx1 or 1xn matrix M as elements
         * @param M
         * @return
         */
        static Matrix diag(const Matrix &M);

        /**
         * @brief extract columns with given index
         * @param idx
         * @return
         */
        const Matrix extract_cols(std::vector<int> idx) const;

        const Matrix row(int i) const;

        static Matrix eye(const int32_t m);

        static Matrix identity(int32_t p, int32_t q);

        void eye();

        void conservative_resize(int32_t p, int32_t q);

        // simple arithmetic operations
        const Matrix operator+(const Matrix &M) const; // add matrix
        const Matrix operator-(const Matrix &M) const; // subtract matrix
        const Matrix operator*(const Matrix &M) const; // multiply with matrix
        const Matrix operator*(const FLOAT &s) const;  // multiply with scalar
        const Matrix operator/(const Matrix &M) const; // divide elementwise by matrix (or vector)
        const Matrix operator/(const FLOAT &s) const;  // divide by scalar

        void operator+=(const Matrix &M) { *this = (*this) + M; }
        const Matrix operator-() const;                        // negative matrix

        const Matrix transpose() const;
        virtual FLOAT l2norm() const;                    // euclidean norm (vectors) / frobenius norm (matrices)
        FLOAT mean();                                    // mean of all elements in matrix

        // complex arithmetic operations
        static Matrix cross(const Matrix &a, const Matrix &b);     // cross product of two vectors
        const Matrix inv(const Matrix &M) const;                        // invert matrix M
        const Matrix inv() const;                                              // invert this matrix
        FLOAT det();                                               // returns determinant of matrix
        bool solve(const Matrix &M, FLOAT eps = 1e-20);            // solve linear system M*x=B, replaces *this and M
        bool lu(int32_t *idx, FLOAT &d, FLOAT eps = 1e-20);        // replace *this by lower upper decomposition
        void svd(Matrix &U, Matrix &W, Matrix &V);                 // singular value decomposition *this = U*diag(W)*V^T
        void ldlt01(Matrix &L, Matrix &D);
        void ldlt(Matrix &L, Matrix &D) const; // ref: https://blog.csdn.net/zhangchao3322218/article/details/7412688

        /**
         * @brief print matrix to stream
         * @param out
         * @param M
         * @return
         */
        friend std::ostream &operator<<(std::ostream &out, const Matrix &M);

    private:
        void allocate_memory(const int32_t m_, const int32_t n_);
        void release_memory();

        inline FLOAT pythag(FLOAT a, FLOAT b);

    private:
        FLOAT **val_;
        int32_t m_, n_;
    };
}

#endif // MATRIX_H
