#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>

int main()
{
	uchar data[] = {0,1,2,3,4,5,6,7,8,9,10,11,8,9,14,15};
	cv::Mat mat_src = cv::Mat(4, 4, CV_8UC1, data);
    std::cout << "mat_src:\n" << mat_src << std::endl;

    cv::Mat mat_hist;
    const int channels = 0;
    const int num_bins = 16;
    const float range_vals[2] = {0.f, 16.f};
    const float *ranges = range_vals;
    cv::calcHist(&mat_src, 1, &channels, cv::noArray(), mat_hist, 1, &num_bins, &ranges);
    std::cout << "mat_hist:\n" << mat_hist << std::endl;
//    cv::normalize(mat_hist, mat_hist, 1,0, cv::NORM_L1);

    cv::Mat mat_project = cv::Mat(4, 4, CV_8UC1);
    cv::Mat array_mat[1] = {mat_src};
    cv::calcBackProject(array_mat, 1, &channels, mat_hist, mat_project, &ranges);
    std::cout << "back project:\n" << mat_project << std::endl;

	return 0;
}

