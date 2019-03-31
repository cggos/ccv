#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

//����̨�ڿ���ʾ
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
	*    ����1�������ͼ��(�����ǲ�ɫͼ��)������2������ı�Եͼ���ǵ�ͨ���ģ������Ǻڰ׵ģ�     *
	*    ����3����һ����ֵ��  ����4���ڶ�����ֵ��  ����5��Sobel�����ں˴�С                     *
	*    ���� cvCanny ����Canny�㷨��������ͼ��ı�Ե���������ͼ���б�ʶ��Щ��Ե��             *
	*    threshold1��threshold2���е�С��ֵ�������Ʊ�Ե���ӣ������ֵ��������ǿ��Ե�ĳ�ʼ�ָ *
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
