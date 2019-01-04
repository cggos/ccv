#include "pointcloud_ros_wrapper.h"

#include <dynamic_reconfigure/server.h>
#include <pointcloud_ros_wrapper/segmentConfig.h>

PointcloudManipulate pcm;

ros::Publisher g_pub;
ros::Publisher g_pub_obstacle;

void ConfigCb(pointcloud_ros_wrapper::segmentConfig &config, uint32_t level) {

    pcm.cam_angle_    = config.cam_angle;
    pcm.roi_frontend_ = config.frontend;
    pcm.roi_backend_  = config.backend;

    ROS_INFO("Dynamicly Set Parameter cam_angle = %f", pcm.cam_angle_);
    ROS_INFO("Dynamicly Set Parameter frontend  = %f", pcm.roi_frontend_);
    ROS_INFO("Dynamicly Set Parameter backend   = %f", pcm.roi_backend_);
}

void CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    switch(cloud_msg->fields.size()) {
        case 3:
            pcl::fromROSMsg(*cloud_msg, *pcm.cloud_input_);
            break;
        case 4: {
            pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb;
            if (0 == cloud_msg->fields[3].name.compare("rgb")) {
                pcl::fromROSMsg(*cloud_msg, cloud_xyzrgb);
            } else {
                ROS_ERROR("cloud_msg is NOT PointXYZRGB!");
                return;
            }
            pcm.cloud_input_->points.resize(cloud_xyzrgb.size());
            pcm.cloud_input_->width  = cloud_xyzrgb.width;
            pcm.cloud_input_->height = cloud_xyzrgb.height;
#pragma omp parallel for
            for (int i = 0; i < cloud_xyzrgb.size(); ++i) {
                pcm.cloud_input_->points[i].x = cloud_xyzrgb.points[i].x;
                pcm.cloud_input_->points[i].y = cloud_xyzrgb.points[i].y;
                pcm.cloud_input_->points[i].z = cloud_xyzrgb.points[i].z;
            }
        }
            break;
        default:
            ROS_ERROR("cloud_msg type NOT compatible, and its fileds size: %d", cloud_msg->fields.size());
    }

    pcm.SegmentForGround();

    sensor_msgs::PointCloud2 cloud_msg_output;
    pcl::toROSMsg(*pcm.cloud_output_, cloud_msg_output);
    cloud_msg_output.header.stamp = ros::Time::now();
    cloud_msg_output.header.frame_id = "cloud_msg_output";
    g_pub.publish(cloud_msg_output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_segment");

    ros::NodeHandle nh;

    dynamic_reconfigure::Server<pointcloud_ros_wrapper::segmentConfig> server;
    dynamic_reconfigure::Server<pointcloud_ros_wrapper::segmentConfig>::CallbackType f = boost::bind(&ConfigCb, _1, _2);
    server.setCallback(f);

    ros::Subscriber sub = nh.subscribe("cloud_in", 1, CloudCb);

    g_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_ground", 1);
    g_pub_obstacle = nh.advertise<sensor_msgs::PointCloud2>("cloud_obstacle", 1);

    ros::Rate loop_rate(30);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

