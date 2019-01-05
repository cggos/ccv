#ifndef CAPTURE_CAM_STEREO_H_
#define CAPTURE_CAM_STEREO_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>

#include "capture_cam/detect_charuco_stereo.h"

namespace capture_cam {

    class CaptureCamStereo : public nodelet::Nodelet {
    public:
        CaptureCamStereo() {}

        virtual ~CaptureCamStereo() {
            if(approx_sync_stereo_)
                delete approx_sync_stereo_;
            if(exact_sync_stereo_)
                delete exact_sync_stereo_;
        }

    private:
        virtual void onInit() {
            ros::NodeHandle &nh  = getNodeHandle();
            ros::NodeHandle &pnh = getPrivateNodeHandle();

            bool approx_sync = true;
            pnh.param("approx_sync", approx_sync, approx_sync);

            int queue_size = 10;
            if(approx_sync) {
                approx_sync_stereo_ = new message_filters::Synchronizer<MyApproxSyncStereoPolicy>(
                        MyApproxSyncStereoPolicy(queue_size), filter_img_l_, filter_img_r_, filter_caminfo_l_,
                        filter_caminfo_r_);
                approx_sync_stereo_->registerCallback(
                        boost::bind(&CaptureCamStereo::stereoCallback, this, _1, _2, _3, _4));
            }
            else {
                exact_sync_stereo_ = new message_filters::Synchronizer<MyExactSyncStereoPolicy>(
                        MyExactSyncStereoPolicy(queue_size), filter_img_l_, filter_img_r_, filter_caminfo_l_,
                        filter_caminfo_r_);
                exact_sync_stereo_->registerCallback(
                        boost::bind(&CaptureCamStereo::stereoCallback, this, _1, _2, _3, _4));
            }

            ros::NodeHandle nh_left  ( nh, "left" );
            ros::NodeHandle nh_right ( nh, "right");
            ros::NodeHandle pnh_left (pnh, "left" );
            ros::NodeHandle pnh_right(pnh, "right");
            image_transport::ImageTransport it_left(nh_left);
            image_transport::ImageTransport it_right(nh_right);
            image_transport::TransportHints hints_left( "raw", ros::TransportHints(), pnh_left);
            image_transport::TransportHints hints_right("raw", ros::TransportHints(), pnh_right);

            filter_img_l_.subscribe(it_left,  nh_left.resolveName("image"),  1, hints_left);
            filter_img_r_.subscribe(it_right, nh_right.resolveName("image"), 1, hints_right);
            filter_caminfo_l_.subscribe(nh_left,  "camera_info", 1);
            filter_caminfo_r_.subscribe(nh_right, "camera_info", 1);

            capture_cam_stereo_imp_ = new StereoCharucoDetector(pnh);
        }

        void stereoCallback(const sensor_msgs::ImageConstPtr& image_left,
                            const sensor_msgs::ImageConstPtr& image_right,
                            const sensor_msgs::CameraInfoConstPtr& caminfo_left,
                            const sensor_msgs::CameraInfoConstPtr& caminfo_right) {
            if (
                    !(image_left->encoding.compare(sensor_msgs::image_encodings::MONO8)  == 0
                    ||image_left->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0
                    ||image_left->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0
                    ||image_left->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
                    !(image_right->encoding.compare(sensor_msgs::image_encodings::MONO8)  == 0
                    ||image_right->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0
                    ||image_right->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0
                    ||image_right->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0)) {
                NODELET_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (enc=%s)", image_left->encoding.c_str());
                return;
            }

            cv_bridge::CvImageConstPtr ptr_img_l, ptr_img_r;
            ptr_img_l = cv_bridge::toCvShare(image_left,  "mono8");
            ptr_img_r = cv_bridge::toCvShare(image_right, "mono8");

            cv::Mat img_l = ptr_img_l->image;
            cv::Mat img_r = ptr_img_r->image;

            capture_cam_stereo_imp_->process(img_l, img_r);
        }

    private:
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyApproxSyncStereoPolicy;
        message_filters::Synchronizer<MyApproxSyncStereoPolicy> * approx_sync_stereo_;

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyExactSyncStereoPolicy;
        message_filters::Synchronizer<MyExactSyncStereoPolicy> * exact_sync_stereo_;

        image_transport::SubscriberFilter filter_img_l_;
        image_transport::SubscriberFilter filter_img_r_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> filter_caminfo_l_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> filter_caminfo_r_;

        CaptureCamStereoImp *capture_cam_stereo_imp_;
    };

    PLUGINLIB_EXPORT_CLASS(capture_cam::CaptureCamStereo, nodelet::Nodelet);
}


#endif // CAPTURE_CAM_STEREO_H_