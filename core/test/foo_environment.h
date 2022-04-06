//
// Created by gordon on 17-12-20.
//

#ifndef CGOCV_FOOENVIRONMENT_H
#define CGOCV_FOOENVIRONMENT_H

#include <gtest/gtest.h>

#include "cgocv/common_cv.h"

#include "CImg.h"
using namespace cimg_library;

class FooEnvironment : public ::testing::Environment {
public:
    virtual void SetUp() {
        mstrPath = "../../data/lena.bmp";

        CImg<unsigned char> img_in(mstrPath.c_str());

        size_.width  = img_in.width();
        size_.height = img_in.height();

        img_gray_ = CImgGrayScale(img_in);
        
        img_gray_.save_bmp("imBW_img.bmp");
    }

    virtual void TearDown() {}

    CImg<unsigned char> CImgGrayScale(CImg<unsigned char> img_in) {
        CImg<unsigned char> img_gray(img_in.width(), img_in.height(), 1, 1);
        cimg_forXY(img_in, x, y) {
            int r = img_in(x, y, 0);
            int g = img_in(x, y, 1);
            int b = img_in(x, y, 2);
            img_gray(x, y, 0) = (r * 0.2126 + g * 0.7152 + b * 0.0722);
        }
        return img_gray;
    }

    std::string mstrPath;
    CImg<unsigned char> img_gray_;
    cg::Size size_;
};

extern FooEnvironment* foo_env;

#endif //CGOCV_FOOENVIRONMENT_H
