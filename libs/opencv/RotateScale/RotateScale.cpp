#include <cmath>
#include <opencv2/opencv.hpp>

int main() {
  IplImage *src = cvLoadImage("../../../data/lena.bmp", 1);
  if (src) {
    IplImage *dst = cvCloneImage(src);
    cvNamedWindow("src", 1);
    cvMoveWindow("src", 200, 200);
    cvShowImage("src", src);

    int angle = 0;
    for (;;) {
      float m[6];
      // Matrix m looks like:
      //
      //[ m0  m1  m2 ] ==> [ A11 A12 b1 ]
      //[ m3  m4  m5 ]     [ A21 A22 b2 ]
      //
      CvMat M = cvMat(2, 3, CV_32F, m);
      int w = src->width;
      int h = src->height;
      int opt = 0;  // 1:旋转缩放  0：旋转
      double factor;
      if (opt)  //旋转加缩放
        factor = (cos(angle * CV_PI / 180) + 1.5) * 2;
      else  //仅仅旋转
        factor = 1;
      m[0] = (float)(factor * cos(angle * CV_PI / 180));
      m[1] = -(float)(factor * sin(angle * CV_PI / 180));
      m[3] = -m[1];
      m[4] = m[0];
      //将旋转中心移至图像中间
      m[2] = w * 0.5f;
      m[5] = h * 0.5f;
      // dst(x,y) = A*src(x,y)+b
      cvGetQuadrangleSubPix(src, dst, &M);
      cvNamedWindow("dst", 1);
      cvMoveWindow("dst", 500, 200);
      cvShowImage("dst", dst);
      if (27 == cvWaitKey(5)) break;
      int delta = 1;
      angle = (int)(angle + delta) % 360;
    }  // for-loop
  }
  return 0;
}
