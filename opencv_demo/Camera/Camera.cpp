#include <opencv2/opencv.hpp>  

using namespace cv;  
using namespace std;  

int main(int argc, char* argv[])
{
	//获取摄像头   
	CvCapture* pCapture = cvCaptureFromCAM(0);  //cvCreateCameraCapture(1)
	if(!pCapture)
	{
		fprintf(stderr,"Could not initialize capturing...\n");
		exit(0);
	}
         
	//捕捉某一帧
	IplImage *pFrame = nullptr;
	pFrame = cvQueryFrame(pCapture);//cvGrabFrame()
	if(pFrame==nullptr)
	{
		cerr<<"Could not grab frame\n\7"<<endl;//转义字符\7报警
	}
	//获得捕捉装置的属性
	int frameH    = (int)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_HEIGHT);
	int frameW    = (int)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_WIDTH );
	int fps       = (int)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FPS);
	int numFrames = (int)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_COUNT);
	cout<<"frameH = "<<frameH<<'\n'
		<<"frameW = "<<frameW<<'\n'
		<<"fps    = "<<fps   <<'\n'
		<<"numFrames="<<numFrames<<endl;

	//初始化视频写入
	CvVideoWriter *writer = NULL;
	int isColor = 1;
	fps     = 25;
	writer = cvCreateVideoWriter("out.avi", CV_FOURCC('D','I','V','X'), fps, cvSize(640,480),isColor);   //XVID  DIVX
	//写入视频文件
	IplImage *img = NULL;
	while(1)
	{
		img = cvRetrieveFrame(pCapture);
		cvWriteFrame(writer, img);
		//在捕捉过程中查看获得的每一帧
		cvShowImage("mainWin", img);
		if(27 == cvWaitKey(20))
			break;
	}
	cvReleaseCapture(&pCapture);
	cvReleaseVideoWriter(&writer);

	////创建窗口   
	//cvNamedWindow("video", 1);  

	////显示视屏   
	//while(1)  
	//{  
	//	pFrame=cvQueryFrame( pCapture );  
	//	if(!pFrame)
	//		break;  
	//	cvShowImage("video",pFrame); 
 // 
 //       //获取帧高度和宽度
 //       int frameH    = (int)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_HEIGHT);
 //       int frameW    = (int)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_WIDTH );
 //       int frameCount = (int)cvGetCaptureProperty(pCapture,CV_CAP_PROP_FRAME_COUNT);
 //       cout<<"frameH = "<<frameH<<'\t'
 //           <<"frameW = "<<frameW<<'\t'
 //           <<"frameCount="<<frameCount<<endl;
 //                    
	//	char c = cvWaitKey(33);  
	//	if(c==27)
	//		break;  
	//}  

	//cvReleaseCapture(&pCapture);  
	//cvDestroyWindow("video"); 

	return 0;
}
