#include "opencv2/video/tracking.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/highgui/highgui.hpp"  
  
#include <iostream>  
#include <ctype.h>  
  
using namespace cv;  
using namespace std;  
  
Mat image;  
  
bool backprojMode = false;  
bool selectObject = false;//用来判断是否选中，当鼠标左键按下时为true，左键松开时为false  
int trackObject = 0;  
bool showHist = true;  
Point origin;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;//图像掩膜需要的边界常数  

static void onMouse( int event, int x, int y, int, void* )  
{  
    if( selectObject )  
    {  
        //选择区域的x坐标选起点与当前点的最小值，保证鼠标不管向右下角还是左上角拉动都正确选择  
        selection.x = MIN(x, origin.x);  
        selection.y = MIN(y, origin.y);    
        selection.width  = std::abs(x - origin.x);  
        selection.height = std::abs(y - origin.y);  
    }  
  
    switch( event )  
    {  
    case CV_EVENT_LBUTTONDOWN:
        origin = Point(x,y);  
        selection = Rect(x,y,0,0);  
        selectObject = true;//这时switch前面的if语句条件为true，执行该语句  
        break;  
    case CV_EVENT_LBUTTONUP:
        selectObject = false;  
        if( selection.width > 0 && selection.height > 0 )  
            trackObject = -1;//重新计算直方图  
        break;  
    }  
}  
  
static void help()
{  
    cout << "\nThis is a demo that shows mean-shift based tracking\n"  
            "You select a color objects such as your face and it tracks it.\n"  
            "This reads from video camera (0 by default, or the camera number the user enters\n"  
            "Usage: \n"  
            "   ./camshiftdemo [camera number]\n";  
  
    cout << "\n\nHot keys: \n"  
            "\tESC - quit the program\n"  
            "\tc - stop the tracking\n"  
            "\tb - switch to/from backprojection view\n"  
            "\th - show/hide object histogram\n"  
            "\tp - pause video\n"  
            "To initialize tracking, select the object with mouse\n";  
}  
  
const char* keys =  
{  
    "{1|  | 0 | camera number}"  
};  
  
int main( int argc, const char** argv )  
{  
    help();  
  
    VideoCapture cap;  
    Rect trackWindow;//要跟踪的窗口  
    int hsize = 16;//创建直方图时要用的常量  
    float hranges[] = {0,180};  
    const float* phranges = hranges;  
    
    CommandLineParser parser(argc, argv, keys);  
    int camNum = parser.get<int>("1");//现在camNum = 0  
    cap.open(camNum);  
    if( !cap.isOpened() )  
    {  
        help();  
        cout << "***Could not initialize capturing...***\n";  
        cout << "Current parameter's value: \n";  
        return -1;  
    }  

    namedWindow( "Histogram", 0 );  
    namedWindow( "CamShift Demo", 0 ); 
    
    setMouseCallback( "CamShift Demo", onMouse, 0 );  

    createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );  
    createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );  
    createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );  
  
    Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(640, 480, CV_8UC3), backproj;  
    
    bool paused = false;  
  
    for(;;)  
    {  
        if( !paused )  
        {  
            cap >> frame;
            if( frame.empty() )
                break;  
        }  
  
        frame.copyTo(image);
  
        if( !paused )  
        {  
            cvtColor(image, hsv, CV_BGR2HSV);//将BGR转换成HSV格式，存入hsv中，hsv是3通道  
  
            if( trackObject )//松开鼠标左键时，trackObject为-1，执行核心部分  
            {  
                int _vmin = vmin, _vmax = vmax;  
  
                //inRange用来检查元素的取值范围是否在另两个矩阵的元素取值之间，返回验证矩阵mask（0-1矩阵）  
                //这里用于制作掩膜板，只处理像素值为H:0~180，S:smin~256, V:vmin~vmax之间的部分。mask是要求的，单通道  
                inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)), Scalar(180, 256, MAX(_vmin, _vmax)), mask);  
  
                int ch[] = {0, 0};
                hue.create(hsv.size(), hsv.depth());//hue是单通道  
                mixChannels(&hsv, 1, &hue, 1, ch, 1);//将H分量拷贝到hue中，其他分量不拷贝。  
  
                if( trackObject < 0 )  
                {  
                    //roi为选中区域的矩阵，maskroi为0-1矩阵  
                    Mat roi(hue, selection), maskroi(mask, selection);  
                    //绘制色调直方图hist，仅限于用户选定的目标矩形区域  
                    calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);  
                    normalize(hist, hist, 0, 255, CV_MINMAX);//必须是单通道，hist是单通道。归一化，范围为0-255  
  
                    trackWindow = selection;  
                    trackObject = 1;//trackObject置1，接下来就不需要再执行这个if块了  
                      
                    histimg = Scalar::all(0);//用于显示直方图
                    int binW = histimg.cols / hsize;//hsize为16，共显示16个  
                    Mat buf(1, hsize, CV_8UC3);
  
                    for( int i = 0; i < hsize; i++ )  
                        //直方图每一项的颜色是根据项数变化的  
                        buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);  
                    cvtColor(buf, buf, CV_HSV2BGR);  
                    //量化等级一共有16个等级，故循环16次，画16个直方块  
                    for( int i = 0; i < hsize; i++ )  
                    {  
                        int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);//获取直方图每一项的高  
                        //画直方图。opencv中左上角为坐标原点  
                        rectangle( histimg, Point(i*binW,histimg.rows),  
                                   Point((i+1)*binW,histimg.rows - val),  
                                   Scalar(buf.at<Vec3b>(i)), -1, 8 );  
                    }  
                }  
                //根据直方图hist计算整幅图像的反向投影图backproj,backproj与hue相同大小  
                calcBackProject(&hue, 1, 0, hist, backproj, &phranges);  
                //计算两个矩阵backproj、mask的每个元素的按位与，返回backproj  
                backproj &= mask;  
                //调用最核心的camshift函数，TermCriteria是算法完成的条件  
                RotatedRect trackBox = CamShift(backproj, trackWindow, TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));  
                if( trackWindow.area() <= 1 )  
                {  
                    int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;  
                    trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,  
                                       trackWindow.x + r, trackWindow.y + r) &  
                                  Rect(0, 0, cols, rows);  
                }  
  
                if( backprojMode )//转换显示方式，将backproj显示出来  
                    cvtColor( backproj, image, CV_GRAY2BGR );  
                //画出椭圆，第二个参数是一个矩形，画该矩形的内接圆  
                ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );  
            }  
        }  
        else if( trackObject < 0 )  
            paused = false;  
  
        if( selectObject && selection.width > 0 && selection.height > 0 )  
        {  
            Mat roi(image, selection);  
            bitwise_not(roi, roi);  
        }  
  
        imshow( "CamShift Demo", image );  
        imshow( "Histogram", histimg );  
  
        //每轮都要等待用户的按键控制  
        char c = (char)waitKey(10);  
        if( c == 27 )//"Esc"键，直接退出  
            break;  
        switch(c)  
        {  
        case 'b'://转换显示方式  
            backprojMode = !backprojMode;  
            break;  
        case 'c'://停止追踪  
            trackObject = 0;  
            histimg = Scalar::all(0);  
            break;  
        case 'h'://隐藏或显示直方图  
            showHist = !showHist;  
            if( !showHist )  
                destroyWindow( "Histogram" );  
            else  
                namedWindow( "Histogram", 1 );  
            break;  
        case 'p'://暂停  
            paused = !paused;//frame停止从摄像头获取图像，只显示旧的图像  
            break;  
        default:  
            ;  
        }  
    }  
  
    return 0;  
}  
