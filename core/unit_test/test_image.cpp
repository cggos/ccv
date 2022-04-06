//
// Created by gordon on 17-12-19.
//
#include "cvkit/image.h"
#include "cvkit/features2d_fast.h"
#include "cvkit/nonmax_suppression.h"

#include "foo_environment.h"

TEST(Image, image)
{
    cg::Image<unsigned char> img(foo_env->size_);
    memcpy(img.data(), foo_env->img_gray_.data(), img.area());
	  int width  = img.size().width;
	  int height = img.size().height;

    cg::Image<unsigned char> imgDst(img);
    cg::Image<unsigned char> imgDstHalf;
    imgDst.half_sample(imgDstHalf);

    cg::Image<unsigned char> imgSample(cg::Size(5,5));
    imgDstHalf.copy(imgSample, 5, 5, cg::Point2D<int>(2,2));

    cg::Image<unsigned char> imgOut;
    img.zoom_with_interpolation(imgOut, 0.3, 0.3, cg::INTERPOLATION_BILINEAR);

	  float degree = 75;
	  float RotationM[3][3] = {0};
    cg::MatrixRotateImage(width/2, height/2, degree/180.f*3.14159, RotationM);
	  unsigned char *imgDstH;
	  unsigned int wDst, hDst;
    cg::ImgProjectiveTransform(img.data(), width, height, RotationM, &imgDstH, &wDst, &hDst);

    CImg<unsigned char> img_bw_out_h(wDst, hDst);
    memcpy(img_bw_out_h.data(), imgDstH, wDst*hDst);
    img_bw_out_h.save_bmp("img_bw_out_h.bmp");
}

TEST(Image, features2d_fast)
{
    cg::Image<unsigned char> imageSrc;
    cg::Image<unsigned char> mimgOrigPatch;
    cg::Features2D::sum_of_squared_distance<int,unsigned char>(imageSrc, mimgOrigPatch);

    cg::Image<unsigned char> img(foo_env->size_);
    memcpy(img.data(), foo_env->img_gray_.data(), img.area());

    cg::FAST fast(cg::Features2D::FAST_9_16);
    std::vector<cg::Point2D<int> > vKeyPts;
    fast.detect_2(img.data(), img.size().width, img.size().height, vKeyPts, 45);

    std::cout << "vKeyPoints.size(): " << vKeyPts.size() << std::endl;

    std::vector<cg::Point2D<int> > vMaxCorners;

    cg::FAST::fast_nonmax(img,vKeyPts,45, vMaxCorners);

    std::cout << "vMaxCorners.size(): " << vMaxCorners.size() << std::endl;

    CImg<unsigned char> img_fastcorner(foo_env->mstrPath.c_str());
    unsigned char green[] = { 0, 255, 0 };
    for (auto &pt : vMaxCorners)
        img_fastcorner.draw_circle(pt.x, pt.y, 1, green);
    img_fastcorner.display("vMaxCorners");
}

TEST(Image, undistort_image)
{
    CImg<unsigned char> img_in("../../data/euroc.png");

    cg::Image<unsigned char> img(img_in.width(), img_in.height());
    memcpy(img.data(), img_in.data(), img.area());
    int width  = img.size().width;
    int height = img.size().height;

    cg::CameraModel  model_cam;
    cg::DistortModel model_dis;
    model_cam.cx =  367.215;
    model_cam.cy =  248.375;
    model_cam.fx =  458.654;
    model_cam.fy =  457.296;
    model_dis.k1 = -0.28340811;
    model_dis.k2 =  0.07395907;
    model_dis.p1 =  0.00019359;
    model_dis.p2 =  1.76187114e-05;

    cg::Image<unsigned char> img_dst(width, height);
    img.undistort_image(img_dst, width, height, model_cam, model_dis);

    CImg<unsigned char> img_dst_out(width, height);
    memcpy(img_dst_out.data(), img_dst.data(), width*height);
    img_dst_out.save_bmp("undistort_image.bmp");
}
