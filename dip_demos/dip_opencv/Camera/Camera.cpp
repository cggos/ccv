#include <opencv2/opencv.hpp>  

using namespace cv;  
using namespace std;  

int main(int argc, char* argv[])
{
	//��ȡ����ͷ   
	CvCapture* pCapture = cvCaptureFromCAM(0);  //cvCreateCameraCapture(1)
	if(!pCapture)
	{
		fprintf(stderr,"Could not initialize capturing...\n");
		exit(0);
	}
         
	//��׽ĳһ֡
	IplImage *pFrame = nullptr;
	pFrame = cvQueryFrame(pCapture);//cvGrabFrame()
	if(pFrame==nullptr)
	{
		cerr<<"Could not grab frame\n\7"<<endl;//ת���ַ�\7����
	}
	//��ò�׽װ�õ�����
	int frameH    = (int)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_HEIGHT);
	int frameW    = (int)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_WIDTH );
	int fps       = (int)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FPS);
	int numFrames = (int)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_COUNT);
	cout<<"frameH = "<<frameH<<'\n'
		<<"frameW = "<<frameW<<'\n'
		<<"fps    = "<<fps   <<'\n'
		<<"numFrames="<<numFrames<<endl;

	//��ʼ����Ƶд��
	CvVideoWriter *writer = NULL;
	int isColor = 1;
	fps     = 25;
	writer = cvCreateVideoWriter("out.avi", CV_FOURCC('D','I','V','X'), fps, cvSize(640,480),isColor);   //XVID  DIVX
	//д����Ƶ�ļ�
	IplImage *img = NULL;
	while(1)
	{
		img = cvRetrieveFrame(pCapture);
		cvWriteFrame(writer, img);
		//�ڲ�׽�����в鿴��õ�ÿһ֡
		cvShowImage("mainWin", img);
		if(27 == cvWaitKey(20))
			break;
	}
	cvReleaseCapture(&pCapture);
	cvReleaseVideoWriter(&writer);

	////��������   
	//cvNamedWindow("video", 1);  

	////��ʾ����   
	//while(1)  
	//{  
	//	pFrame=cvQueryFrame( pCapture );  
	//	if(!pFrame)
	//		break;  
	//	cvShowImage("video",pFrame); 
 // 
 //       //��ȡ֡�߶ȺͿ��
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
