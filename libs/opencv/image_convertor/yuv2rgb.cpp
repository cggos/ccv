#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// download image_yuv_nv21_1280_800_01.raw
// from https://download.csdn.net/download/u011178262/10791506

void yuv_nv21_to_rgb(unsigned char argb[], char yuv[], int width, int height);

int main()
{
    const int width  = 1280;
    const int height = 800;

    std::ifstream file_in;
    file_in.open("../../../data/image_yuv_nv21_1280_800_01.raw", std::ios::binary);
    std::filebuf *p_filebuf = file_in.rdbuf();
    size_t size = p_filebuf->pubseekoff(0, std::ios::end, std::ios::in);
    p_filebuf->pubseekpos(0, std::ios::in);

    char *buf_src = new char[size];
    p_filebuf->sgetn(buf_src, size);

    cv::Mat mat_dst = cv::Mat(height, width, CV_8UC3);
#if 0 // good
    cv::Mat mat_src = cv::Mat(height*1.5, width, CV_8UC1, buf_src);
    cv::cvtColor(mat_src, mat_dst, cv::COLOR_YUV2BGR_NV21);
#else // poor
    unsigned char *buf_dst = new unsigned char[width*height*3];
    yuv_nv21_to_rgb(buf_dst, buf_src, width, height);
    memcpy(mat_dst.data, buf_dst, width*height*3);
    delete []buf_dst;
#endif
    cv::imwrite("yuv.png", mat_dst);

    file_in.close();
    delete []buf_src;

    return 0;
}

void yuv_nv21_to_rgb(unsigned char rgb[], char yuv[], int width, int height) {

    int total = width * height;
    char Y, U, V;
    unsigned char R, G, B;
    int index = 0;

    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {

            Y = yuv[h * width + w];
            if ((w & 1) == 0)
                V = yuv[total + (h >> 1) * width + w];
            if ((w & 1) == 1)
                U = yuv[total + (h >> 1) * width + w - 1];

            // OpenCV YCrCb --> RGB
            //B = Y + 1.773*(U-128);
            //G = Y - 0.714*(V-128) - 0.344*(U-128);
            //R = Y + 1.403*(V-128);

            // YUV-->RGB for HDTV(BT.601)
            //B = Y + 2.03211*(U-128);
            //G = Y - 0.39465*(U-128) - 0.5806*(V-128);
            //R = Y + 1.13983*(V-128);

            // YUV-->RGB for HDTV(BT.709)
            //B = Y + 2.12798*(U-128);
            //G = Y - 0.21482*(U-128) - 0.38059*(V-128);
            //R = Y + 1.28033*(V-128);

            // YCbCr-->RGB
            B = 1.164*(Y-16) + 2.018*(U-128);
            G = 1.164*(Y-16) - 0.813*(U-128) - 0.391*(V-128);
            R = 1.164*(Y-16) + 1.596*(V-128);

            if (R < 0) R = 0; else if (R > 255) R = 255;
            if (G < 0) G = 0; else if (G > 255) G = 255;
            if (B < 0) B = 0; else if (B > 255) B = 255;

            rgb[index++] = B;
            rgb[index++] = G;
            rgb[index++] = R;
        }
    }
}
