#include <opencv2/opencv.hpp>  
#include <iomanip>
using namespace cv;  
using namespace std;  

int main()
{
    //RGB
    cout<<'\n'<<setw(60)<<"RGB"<<endl;
    cout<<setw(85)<<"ע����ÿ�е���߿�ʼ��ÿ����Ϊһ�� R G B ֵ��"<<endl;

    const string strName = "..\\res\\gray.jpg";
	IplImage *img2=cvLoadImage(strName.c_str(),-1);
    if(!img2)
    {
        cout<<"ͼƬ����ʧ�ܣ�"<<endl;
        system("pause");
        return 0;
    }
    int h = img2->height;
    int w = img2->width;
    
    //��������ͼƬ����ָ��h/10�ݣ�����ֳ�w/32��
    for(int y=0; y<h; y+=10)
    {
        for(int x=0; x<w; x+=32)
        {
            //�������õ�ÿ�ݵ�BGR��ɫֵ
            CvScalar color = cvGet2D(img2,y,x);
            //�� R G B ��˳�����
            cout<<setw(4)<<color.val[2]<<setw(4)<<color.val[1]<<setw(4)<<color.val[0]<<"  ";
        }
        cout<<endl;
    }
    cvNamedWindow(strName.c_str());
    cvShowImage(strName.c_str(),img2);
    cvWaitKey();
    return 0;
}
