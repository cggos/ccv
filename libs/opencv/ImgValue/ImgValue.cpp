#include <opencv2/opencv.hpp>  
#include <iomanip>
using namespace cv;  
using namespace std;  

int main()
{
    //RGB
    cout<<'\n'<<setw(60)<<"RGB"<<endl;
    cout<<setw(85)<<"注：从每行的左边开始，每三个为一个 R G B 值！"<<endl;

    const string strName = "..\\res\\gray.jpg";
	IplImage *img2=cvLoadImage(strName.c_str(),-1);
    if(!img2)
    {
        cout<<"图片加载失败！"<<endl;
        system("pause");
        return 0;
    }
    int h = img2->height;
    int w = img2->width;
    
    //采样：将图片纵向分割成h/10份，横向分成w/32份
    for(int y=0; y<h; y+=10)
    {
        for(int x=0; x<w; x+=32)
        {
            //量化：得到每份的BGR颜色值
            CvScalar color = cvGet2D(img2,y,x);
            //按 R G B 的顺序输出
            cout<<setw(4)<<color.val[2]<<setw(4)<<color.val[1]<<setw(4)<<color.val[0]<<"  ";
        }
        cout<<endl;
    }
    cvNamedWindow(strName.c_str());
    cvShowImage(strName.c_str(),img2);
    cvWaitKey();
    return 0;
}
