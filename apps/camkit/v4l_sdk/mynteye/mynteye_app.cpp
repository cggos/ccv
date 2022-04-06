#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "mynteye_api.h"

using namespace std;

int main() {
  cout << "please input the video device index, e.g. 0, 1: ";
  int dex;
  cin >> dex;
  cout << endl;

  MyntEyeCam cam(dex);

  cam.get_cap();

  while (cam.is_opened()) {
    cam.read();

    cv::Mat image_all;
    cv::hconcat(cam.get_img_l(), cam.get_img_r(), image_all);
    cv::imshow("left+right", image_all);

    char key = cv::waitKey(1);
    if (key == 27 || key == 'q' | key == 'Q') {
      break;
    }
  }

  return 0;
}