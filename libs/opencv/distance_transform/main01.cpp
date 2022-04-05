#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int main() {
  //初始化输入图像和变换结果图像
  Mat mat(480, 480, CV_8UC1, Scalar(0)), transMatE, transMatD4, transMatD8;

  //给输入图像指定三个像素点作为距离变换原点(区域块)
  mat.at<uchar>(100, 200) = 1;
  mat.at<uchar>(200, 100) = 1;
  mat.at<uchar>(300, 300) = 1;

  //将将输入图像中1和0调换，使得原点距离为0
  mat = 1 - mat;

  //显示原始图像(显示为黑色)
  imshow("原始图片", mat);

  //分别利用欧式距离、D4距离和D8距离作距离变换，将结果存入transMatD4、transMatD8和transMatE
  distanceTransform(mat, transMatE, DIST_L2, 0);
  distanceTransform(mat, transMatD4, DIST_L1, 0, CV_8U);
  distanceTransform(mat, transMatD8, DIST_C, 0);

  //欧式距离与D8距离作变换后，值为32位浮点数，以下代码将其值转为uchar类型
  transMatE.convertTo(transMatE, CV_8U);
  transMatD8.convertTo(transMatD8, CV_8U);

  //显示距离变换结果
  imshow("欧式距离变换后的图片", transMatE);
  imshow("D4距离变换后的图片", transMatD4);
  imshow("D8距离变换后的图片", transMatD8);

  waitKey();

  return 0;
}