#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

//控制台黑框不显示
#pragma comment( linker, "/subsystem:windows /entry:mainCRTStartup" )

int varThreshold = 1;
IplImage *image = 0, *grayImage = 0, *edgeImage = 0, *outImage = 0;
void on_trackbar(int h);
int main()
{
    image = cvLoadImage("..\\resources\\2.jpg", 1);
    if(!image)
    {
        cout<<"Loading image is failed!"<<endl;
        return -1;
    }
    cvNamedWindow("image", 1);
    cvShowImage("image", image);

    //Create the color image, grey scale image and output image
    grayImage = cvCreateImage(cvSize(image->width,image->height), IPL_DEPTH_8U, 1);
    edgeImage = cvCreateImage(cvSize(image->width,image->height), IPL_DEPTH_8U, 1);
    outImage  = cvCreateImage(cvSize(image->width,image->height), IPL_DEPTH_8U, 3); 

    //Create a window
    cvNamedWindow("EdgeDetection", 1);
    //Create a toolbar
    cvCreateTrackbar("Threshold", "EdgeDetection", &varThreshold, 100, on_trackbar);
    //Set the initial position of trackbar
    cvSetTrackbarPos("Threshold", "EdgeDetection", 50);

    //Show the image
//    on_trackbar(0);

    //Wait for a key stroke;the same function arranges events processing 
    cvWaitKey(0);
    cvReleaseImage(&image);
    cvReleaseImage(&grayImage);
    cvReleaseImage(&edgeImage);
    cvDestroyWindow("Toolbar");
    return 0;
}
//Definite the callback function of the trackbar
void on_trackbar(int h)
{
	/********************************************************************************************
	*                                      cvCanny()                                            *                           
	*    参数1：输入的图像(可以是彩色图像)；参数2：输出的边缘图像是单通道的，但是是黑白的；     *
	*    参数3：第一个阈值；  参数4：第二个阈值；  参数5：Sobel算子内核大小                     *
	*    函数 cvCanny 采用Canny算法发现输入图像的边缘而且在输出图像中标识这些边缘。             *
	*    threshold1和threshold2当中的小阈值用来控制边缘连接，大的阈值用来控制强边缘的初始分割。 *
	********************************************************************************************/

    cvSmooth(grayImage, edgeImage, CV_BLUR, 3, 3, 0);
    cvNot(grayImage, edgeImage);
    
    //Convert the color image into grey scale image
    cvCvtColor(image, grayImage, CV_BGR2GRAY); 
    //Detect the edgeImages of grey scale image
    cvCanny(grayImage, edgeImage, (float)varThreshold, (float)varThreshold*3, 3);
    //Convert the grey scale image into color image
    cvCvtColor(edgeImage, outImage, CV_GRAY2BGR); 

    cvZero(outImage);
    //Copy edgeImage points
    cvCopy(image, outImage, edgeImage);
    cvShowImage("EdgeDetection", outImage);
}
