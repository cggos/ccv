//
// Created by cg on 9/17/19.
//

#ifndef VIKIT_CG_YIMG_H
#define VIKIT_CG_YIMG_H

#include <string.h>
#include <stdint.h>

#include "common.h"
#include "cv/types.h"

namespace cg {

    template<class _T>
    class YImg {
    public:
        YImg() : size_(0, 0), data_(nullptr) {}

        YImg(unsigned int rows, unsigned int cols) {
            allocate_memory(rows, cols);
        }

        YImg(const Size &size) {
            allocate_memory(size.height, size.width);
        }

        YImg(const YImg &img) {
            allocate_memory(img.rows(), img.cols());
            for(int i=0; i<img.rows(); ++i)
                memcpy(data_[i], img.data_[i], img.cols() * sizeof(_T));
        }

        ~YImg() {
            release_memory();
        }

        YImg &operator=(const YImg &rhs) {
            if (this == &rhs)
                return *this;

            if(!rhs.empty())
                release_memory();

            allocate_memory(rhs.rows(), rhs.cols());
            for(int i=0; i<rhs.rows(); ++i)
                memcpy(data_[i], rhs.data_[i], rhs.cols() * sizeof(_T));
            
            return *this;
        }

        _T &operator()(const int i, const int j) const { return data_[i][j]; }

        _T &operator[](const Point2D<int> &pt) const { return data_[pt.y][pt.x]; }

        inline _T *data() const { return data_[0]; }

        inline bool empty() const { return data_ == nullptr; }

        inline Size size() const { return size_; }

        inline int rows() const { return size().height; }

        inline int cols() const { return size().width; }

        bool copy(YImg &img_dst) {
            if (img_dst.empty())
                return false;
            if (img_dst.size() != size())
                return false;
            for (int i = 0; i < rows(); ++i)
                memcpy(img_dst.data_[i], data_[i], cols() * sizeof(_T));
            return true;
        }

    private:
        void allocate_memory(const int rows, const int cols) {
            size_.height = rows;
            size_.width = cols;

            if (cols == 0 || rows == 0) {
                data_ = nullptr;
                return;
            }

            data_ = new _T*[size_.height];
            data_[0] = new _T[size_.area()]();
            for (int i = 1; i < size_.height; i++)
                data_[i] = data_[i - 1] + size_.width;
        }

        void release_memory() {
            if (data_ != nullptr) {
                delete[] data_[0];
                delete[] data_;
            }
        }

    private:
        Size size_;
        _T **data_;
    };

    typedef YImg<unsigned char> YImg8;
    typedef YImg<unsigned short> YImg16;
    typedef YImg<float> YImg32f;
}

#endif //VIKIT_CG_YIMG_H
