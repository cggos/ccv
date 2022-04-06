//
// Created by cg on 3/11/19.
//

#include "cam_v4l2.h"

int main()
{
    cg::CamV4l2 cam_v4l2("/dev/video0", 640, 480);

    while(true) {
        cv::Mat mat_dst = cam_v4l2.read();

        cv::imshow("live cam", mat_dst);
        int key = cv::waitKey(30);
        if(key == 27)
            break;
    }

    return 0;
}
