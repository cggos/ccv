#include <sensor_msgs/Imu.h>

#include "pointcloud_ros_wrapper.h"

PointcloudManipulate pcm;

ros::Publisher g_pub;

void ImuCb(const sensor_msgs::Imu::ConstPtr &msg) {
    pcm.quaternion_imu_ = msg->orientation;
}

void CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    pcl::fromROSMsg(*cloud_msg, *pcm.cloud_rgb_input_);

    pcm.TransformByImu();

    sensor_msgs::PointCloud2 cloud_msg_output;
    pcl::toROSMsg(*pcm.cloud_rgb_output_, cloud_msg_output);
    cloud_msg_output.header = cloud_msg->header;

    g_pub.publish(cloud_msg_output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_transform_imu");

    ros::NodeHandle nh;

    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::Imu>("imu_data", 10, boost::bind(&ImuCb, _1));

    ros::Subscriber sub_imu   = nh.subscribe("cloud_in", 1, CloudCb);

    g_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);

    ros::Rate loop_rate(30);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
