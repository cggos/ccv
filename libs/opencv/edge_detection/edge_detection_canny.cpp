#include <iostream>
#include <opencv2/core/utility.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>

using namespace cv;
using namespace std;

static const char* keys = {"{@image_path | | Image path }"};

static void help() {
  cout << "\nThis example shows the functionalities of lines extraction "
       << "furnished by BinaryDescriptor class\n"
       << "Please, run this sample using a command in the form\n"
       << "./line_detect <path_to_input_image>" << endl;
}

int main(int argc, char** argv) {
  CommandLineParser parser(argc, argv, keys);
  String image_path = parser.get<String>(0);

  if (image_path.empty()) {
    help();
    return -1;
  }

  cv::Mat imageMat = imread(image_path, 1);
  if (imageMat.data == NULL) {
    std::cout << "Error, image could not be loaded. Please, check its path" << std::endl;
    return -1;
  }

  cv::Mat output;

  cv::Canny(imageMat, output, 150, 100);

  imshow("LSD lines", output);
  waitKey();
}