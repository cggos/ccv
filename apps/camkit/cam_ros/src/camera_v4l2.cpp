#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "cam_v4l2.h"

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "camera");

    ros::NodeHandle pnh("~");

    std::string device_name = "/dev/video0";
    pnh.param<std::string>("device", device_name, device_name);

    image_transport::ImageTransport it(pnh);
    image_transport::Publisher pub_image_raw = it.advertise("image_raw", 1);

    cv_bridge::CvImagePtr frame;
    frame = boost::make_shared<cv_bridge::CvImage>();
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    cg::CamV4l2 cam_v4l2(device_name, 640, 480);

    while (ros::ok()) {
        frame->image = cam_v4l2.read();

        if (not frame->image.empty()) {
            frame->header.stamp = ros::Time::now();
            pub_image_raw.publish(frame->toImageMsg());
        }
    }

    return EXIT_SUCCESS;
}
