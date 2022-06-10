#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

struct userdata {
  Mat im;
  vector<Point2f> points;
};

void mouseHandler(int event, int x, int y, int flags, void* data_ptr) {
  if (event == EVENT_LBUTTONDOWN) {
    userdata* data = ((userdata*)data_ptr);
    circle(data->im, Point(x, y), 3, Scalar(0, 255, 255), 5, CV_AA);
    imshow("Image", data->im);
    if (data->points.size() < 4) {
      data->points.push_back(Point2f(x, y));
    }
  }
}

int main(int argc, char** argv) {
  Mat im_src = imread("../../../data/lena.bmp");
  Mat im_dst = imread("../../../data/ad.jpg");

  if (im_src.empty() || im_dst.empty()) {
    std::cerr << "ERROR: img empty!" << std::endl;
    return -1;
  }

  vector<Point2f> pts_src;
  Size size = im_src.size();
  pts_src.push_back(Point2f(0, 0));
  pts_src.push_back(Point2f(size.width - 1, 0));
  pts_src.push_back(Point2f(size.width - 1, size.height - 1));
  pts_src.push_back(Point2f(0, size.height - 1));

  cout << "Click on four corners of a billboard and then press ENTER" << endl;
  Mat im_temp = im_dst.clone();
  userdata data;
  data.im = im_temp;
  imshow("Image", im_temp);
  setMouseCallback("Image", mouseHandler, &data);
  waitKey(0);

  Mat H = findHomography(pts_src, data.points, 0);

  Mat logo;
  warpPerspective(im_src, logo, H, im_dst.size());
  Mat log_warp = logo.clone();

  Mat logo_mask;
  threshold(logo, logo_mask, 0, 255, cv::ThresholdTypes::THRESH_BINARY_INV);
  im_dst.copyTo(logo, logo_mask);  // im_dst & logo_mask --> im_dst + logo --> logo

  imshow("ad", logo);

  cv::resize(im_dst, im_dst, im_dst.size() / 2);
  cv::resize(log_warp, log_warp, log_warp.size() / 2);
  cv::resize(logo, logo, logo.size() / 2);
  cv::resize(logo_mask, logo_mask, logo_mask.size() / 2);
  cv::Mat img_h1, img_h2;
  cv::hconcat(im_dst, log_warp, img_h1);
  cv::hconcat(logo, logo_mask, img_h2);
  cv::Mat img_all;
  cv::vconcat(img_h1, img_h2, img_all);
  imshow("img_all", img_all);

  waitKey(0);

  return 0;
}
