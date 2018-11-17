#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main()
{
    std::ifstream file_in;
    file_in.open("../image_yuv_nv21_1280_800_01.raw", std::ios::binary);
    std::filebuf *p_filebuf = file_in.rdbuf();
    size_t size = p_filebuf->pubseekoff(0, std::ios::end, std::ios::in);
    p_filebuf->pubseekpos(0, std::ios::in);

    char *buffer = new char[size];
    p_filebuf->sgetn(buffer, size);

    cv::Mat mat_src = cv::Mat(800*1.5, 1280, CV_8UC1, buffer);
    cv::Mat mat_dst = cv::Mat(800, 1280, CV_8UC3);
    cv::cvtColor(mat_src, mat_dst, cv::COLOR_YUV2BGR_NV21);

    cv::imwrite("yuv.png", mat_dst);

    file_in.close();
    delete []buffer;

    return 0;
}
