#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include <image_transport/image_transport.h>

#include "mipi_camera.hpp"

class MiPiWrapperNode {
public:
    MiPiWrapperNode() : pnh("~") {
        onInit();
    }

    void device_poll() {
        timeval time_val1, time_val2;
        cv::Mat frame1, frame2;
        camera1.read_frame(&time_val1, frame1);
        camera2.read_frame(&time_val2, frame2);

        // sync
        if (mipi::frame_diff(&time_val1, &time_val2) > mipi::kTPF) {
            if (!camera1.synctime(&time_val2, &time_val1, frame1))
                return;
        } else if (mipi::frame_diff(&time_val1, &time_val2) < -mipi::kTPF) {
            if (!camera2.synctime(&time_val1, &time_val2, frame2))
                return;
        }

        double t1 = time_val1.tv_sec + time_val1.tv_usec * mipi::kMicroScale;
        double t2 = time_val2.tv_sec + time_val2.tv_usec * mipi::kMicroScale;

        // std::cout << "====: " << std::abs(t1-t2)*1000 << std::endl;

        publishImage(pub_raw_left,  frame1, sensor_msgs::image_encodings::MONO8, left_frame_id,  ros::Time(t1));
        publishImage(pub_raw_right, frame2, sensor_msgs::image_encodings::MONO8, right_frame_id, ros::Time(t2));

        // pub transform
        static tf::TransformBroadcaster br;
        tf::Transform tf;
        tf.setIdentity();
        br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "mynteye_link", left_frame_id));
        br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "mynteye_link", right_frame_id));
    }

private:
    void onInit() {
        std::string left_image_raw_topic  = "left_image_raw_topic";
        std::string right_image_raw_topic = "right_image_raw_topic";
        left_frame_id  = "left_frame_id";
        right_frame_id = "right_frame_id";

        pnh.param("left_image_raw_topic",  left_image_raw_topic, left_image_raw_topic);
        pnh.param("right_image_raw_topic", right_image_raw_topic, right_image_raw_topic);
        pnh.param("left_frame_id", left_frame_id, left_frame_id);
        pnh.param("right_frame_id", right_frame_id, right_frame_id);

        pub_raw_left  = nh.advertise<sensor_msgs::Image>(left_image_raw_topic, 1);
        pub_raw_right = nh.advertise<sensor_msgs::Image>(right_image_raw_topic, 1);

        camera1.open_camera(0);
        camera2.open_camera(1);

        // default throw 10 frames
        camera1.throw_some_frames(10);
        camera2.throw_some_frames(10);
    }

    void publishImage(ros::Publisher &pub_img, const cv::Mat &img, const std::string encodingType, const std::string &frame_id, ros::Time stamp) {
        if (pub_img.getNumSubscribers() == 0)
            return;
        static uint32_t seq = 0;
        std_msgs::Header header;
        header.seq = seq;
        header.stamp = stamp;
        header.frame_id = frame_id;
        auto &&msg = cv_bridge::CvImage(header, encodingType, img);
        pub_img.publish(msg.toImageMsg());
        ++seq;
    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Publisher pub_raw_left;
    ros::Publisher pub_raw_right;

    std::string right_frame_id;
    std::string left_frame_id;

    mipi::Camera camera1;
    mipi::Camera camera2;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mini_ros_wrapper_node");

    MiPiWrapperNode camera_node;
    while (ros::ok()) {
        camera_node.device_poll();
    }

    return 0;
}
