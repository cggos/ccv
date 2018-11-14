#include <iostream>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main()
{
	IplImage* src=cvLoadImage(".\\li_3.jpg",0);  //..\\resources
	int width=src->width;  
	int height=src->height;  
	int step=src->widthStep;  
	uchar* data=(uchar *)src->imageData;  
	int hist[256]={0};  
	for(int i=0;i<height;i++)  
	{  
		for(int j=0;j<width;j++)  
		{  
			hist[data[i*step+j]]++;
		}  
	}  
	int max=0;  
	for(int i=0;i<256;i++)  
	{  
		if(hist[i]>max)  
		{  
			max=hist[i];  
		}  
	}  
	IplImage* dst=cvCreateImage(cvSize(400,300),8,3);  
	cvSet(dst,cvScalarAll(255),0);  
	double bin_width=(double)dst->width/256;  
	double bin_unith=(double)dst->height/max;  
	for(int i=0;i<256;i++)  
	{  
		CvPoint p0=cvPoint(i*bin_width,dst->height);  
		CvPoint p1=cvPoint((i+1)*bin_width,dst->height-hist[i]*bin_unith);  
		cvRectangle(dst,p0,p1,cvScalar(0,255),-1,8,0);  
	}  

	double bw = bin_width*16;
	for (int j=0;j<16;j++)
	{
		CvPoint p0=cvPoint(j*bw,dst->height);  
		CvPoint p1=cvPoint(j*bw+bin_width,dst->height-bin_width);  
		cvRectangle(dst,p0,p1,cvScalar(0,0,255),-1,8,0);
	}

	cvNamedWindow("src",1);  
	cvShowImage("src",src);  
	cvNamedWindow("dst",1);  
	cvShowImage("dst",dst);
	cvSaveImage("li_3.bmp",dst);

	cvWaitKey(0); 

	cvDestroyAllWindows();  
	cvReleaseImage(&src);  
	cvReleaseImage(&dst);   

	return 0;
}

