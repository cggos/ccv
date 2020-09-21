#include <linux/videodev2.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/videoio/videoio.hpp>

#define S1030  // S1030 D1000

using namespace std;
using namespace cv;

int main() {
    cout << "please input the video device index, e.g. 0, 1: ";
    int dex;
    cin >> dex;
    cout << endl;

    VideoCapture cap;
    cap.open(dex);
    if (!cap.isOpened()) {
        cerr << "open cap failed." << endl;
        return 1;
    }

#ifdef S1030
    cap.set(cv::CAP_PROP_CONVERT_RGB, 0);  // !!!
#endif

#ifdef D1000
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 2560);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    // cap_depth.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    // cap_depth.set(CV_CAP_PROP_FRAME_HEIGHT,720);
#endif

    unsigned int fmt = cap.get(CV_CAP_PROP_FORMAT);
    unsigned int mode = cap.get(CAP_PROP_MODE);
    int w = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int h = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    int fps = cap.get(CV_CAP_PROP_FPS);

    std::string str_mode = "";
    switch (mode) {
        case V4L2_PIX_FMT_BGR24:
            str_mode = "FMT_BGR24";
            break;
        case V4L2_PIX_FMT_YVU420:
            str_mode = "FMT_YVU420";
            break;
        case V4L2_PIX_FMT_YUV411P:
            str_mode = "FMT_YUV411P";
            break;
        case V4L2_PIX_FMT_MJPEG:
            str_mode = "FMT_MJPG";
            break;
        case V4L2_PIX_FMT_JPEG:
            str_mode = "FMT_JPG";
            break;
        case V4L2_PIX_FMT_YUYV:
            str_mode = "FMT_YUYV";
            break;
        case V4L2_PIX_FMT_RGB24:
            str_mode = "FMT_RGB24";
            break;
        case V4L2_PIX_FMT_UYVY:
            str_mode = "FMT_UYVY";
            break;
        default:
            str_mode = "mode not found!";
            break;
    }

    cout << "cap format: " << fmt << endl;
    cout << "cap mode: " << mode << ", " << str_mode << endl;
    cout << "cap frame width:  " << w << endl;
    cout << "cap frame height: " << h << endl;
    cout << "cap frame: " << fps << endl;

    int size = w * h;
    while (true) {
        Mat raw;
        cap.read(raw);

        Mat left, right;

#ifdef S1030
        left = Mat(raw.size(), CV_8UC1);
        right = Mat(raw.size(), CV_8UC1);
        for (int i = 0; i < raw.rows * raw.cols; ++i) {
            left.data[i] = raw.data[i * 2];
            right.data[i] = raw.data[i * 2 + 1];
        }
#endif

#ifdef D1000
        //color image(1280*720 * 2)
        // left = Mat(h, w/2, CV_8UC3);
        // right = Mat(h, w/2, CV_8UC3);
        const int w1 = 720;
        left = Mat(h, w1, CV_8UC3);
        right = Mat(h, w1, CV_8UC3);
        //gray image
        // cvtColor(raw,raw,CV_BGR2GRAY);
        // left = Mat(h,w/2,CV_8UC1);
        // right = Mat(h,w/2,CV_8UC1);

        //use ptr
        // //left frame
        // for (int i = 0; i < h ; i++)
        // {
        //     uchar *ptr = raw.ptr<uchar> (i);
        //     for( int j = 0; j < (w * 3) / 2; j++)
        //     left.ptr<uchar>(i) [j] = ptr[j];
        // }

        // //right frame
        // for (int i = 0; i < h; i++)
        // {
        //     uchar *ptr = raw.ptr<uchar> (i);
        //     for( int j = (w * 3) /2 ; j < (w * 3); j++ )
        //     right.ptr<uchar>(i) [j] = ptr[j];
        // }

        //use ROI

        //use color image,only use width and height,don't need to consider channel
        left = raw(Rect((w / 4 - w1 / 2), 0, (w / 4 + w1 / 2), h));
        right = raw(Rect((3 * w / 4 - w1 / 2), 0, (3 * w / 4 + w1 / 2), h));

        //use gray image
        // left = raw(Rect(0,0,w/2,h));
        // right = raw(Rect(w/2,0,w/2 ,h));

        //use left image to dectect pedestrian
        vector<Rect> found;

        HOGDescriptor defaultHog;
        defaultHog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
        defaultHog.detectMultiScale(left, found);

        for (int i = 0; i < found.size(); i++) {
            Rect r = found[i];
            rectangle(left, r, Scalar(0, 0, 255));
        }

        // Mat depth(h_d,w_d,CV_16UC1);
        // cap_depth.read(depth);
        // cout << depth <<endl;
        //imshow("depth",depth);
        //imshow("raw",raw);
#endif

        Mat image_all;
        hconcat(left, right, image_all);
        imshow("left+right", image_all);

        char key = static_cast<char>(waitKey(1));

        if (key == 32 || key == 's') {
            static int32_t count = 0;
            count++;
            cout << count << endl;
            stringstream ss_l, ss_r;
            ss_l << "left" << (count) << ".png";
            ss_r << "right" << (count) << ".png";
            string lfilename = ss_l.str();
            string rfilename = ss_r.str();
            imwrite(lfilename, left);
            imwrite(rfilename, right);
            cout << "save " << lfilename << endl;
            cout << "save " << rfilename << endl;
        }

        if (key == 27 || key == 'q' | key == 'Q') {
            break;
        }
    }

    return 0;
}