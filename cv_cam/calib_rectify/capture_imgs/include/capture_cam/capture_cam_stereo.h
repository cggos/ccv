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

#include "detect_charuco/detect_charuco_stereo.h"
#include "detect_checker/detect_checker_stereo.h"

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
                        MyApproxSyncStereoPolicy(queue_size), filter_img_l_, filter_img_r_);
                approx_sync_stereo_->registerCallback(
                        boost::bind(&CaptureCamStereo::stereoCallback, this, _1, _2));
            }
            else {
                exact_sync_stereo_ = new message_filters::Synchronizer<MyExactSyncStereoPolicy>(
                        MyExactSyncStereoPolicy(queue_size), filter_img_l_, filter_img_r_);
                exact_sync_stereo_->registerCallback(
                        boost::bind(&CaptureCamStereo::stereoCallback, this, _1, _2));
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

            ros::NodeHandle nh_rect_l(nh, "left_rect" );
            ros::NodeHandle nh_rect_r(nh, "right_rect");
            image_transport::ImageTransport it_l(nh_rect_l);
            image_transport::ImageTransport it_r(nh_rect_r);
            // rect_pub_l_ = it_l.advertiseCamera("image", 1, false);
            // rect_pub_r_ = it_r.advertiseCamera("image", 1, false);

            rect_it_pub_l_      = it_l.advertise("image", 1);
            rect_it_pub_r_      = it_r.advertise("image", 1);
            rect_caminfo_pub_l_ = nh_rect_l.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
            rect_caminfo_pub_r_ = nh_rect_r.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

            std::string str_params_dir = "./params_out";
            pnh.param("param_dir",  str_params_dir, str_params_dir);
            capture_cam_stereo_imp_ = new StereoCheckerDetector(pnh, str_params_dir);
        }

        void stereoCallback(const sensor_msgs::ImageConstPtr& image_left,
                            const sensor_msgs::ImageConstPtr& image_right) {
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

            if(capture_cam_stereo_imp_->is_rectify_)
                pub_img_rect(capture_cam_stereo_imp_->img_rect_l_,
                             capture_cam_stereo_imp_->img_rect_r_,
                             ptr_img_l->header, ptr_img_r->header);
        }

        void pub_img_rect(
                const cv::Mat &img_l, const cv::Mat &img_r,
                const std_msgs::Header &header_l,
                const std_msgs::Header &header_r) {

            sensor_msgs::Image img_msg_l;
            cv_bridge::CvImage(header_l, sensor_msgs::image_encodings::MONO8, img_l).toImageMsg(img_msg_l);

            sensor_msgs::Image img_msg_r;
            cv_bridge::CvImage(header_r, sensor_msgs::image_encodings::MONO8, img_r).toImageMsg(img_msg_r);

            sensor_msgs::CameraInfo cam_info_l;
            cam_info_l.header = img_msg_l.header;
            cam_info_l.width  = img_l.cols;
            cam_info_l.height = img_l.rows;
            cam_info_l.distortion_model = "";

            cam_info_l.D.resize(4, 0);
            for(int i=0; i<4; ++i)
                cam_info_l.D[i] = 0;

            for(int i=0; i<9; ++i)
                cam_info_l.K[i] = ((double*)capture_cam_stereo_imp_->K1_.data)[i];

            for(int i=0; i<9; ++i)
                cam_info_l.R[i] = ((double*)capture_cam_stereo_imp_->R1_.data)[i];

            for(int i=0; i<12; ++i)
                cam_info_l.P[i] = ((double*)capture_cam_stereo_imp_->P1_.data)[i];


            sensor_msgs::CameraInfo cam_info_r;
            cam_info_r.header = img_msg_l.header;
            cam_info_r.width  = img_r.cols;
            cam_info_r.height = img_r.rows;
            cam_info_r.distortion_model = "";

            cam_info_r.D.resize(4);
            for(int i=0; i<4; ++i)
                cam_info_r.D[i] = 0;

            for(int i=0; i<9; ++i)
                cam_info_r.K[i] = ((double*)capture_cam_stereo_imp_->K2_.data)[i];

            for(int i=0; i<9; ++i)
                cam_info_r.R[i] = ((double*)capture_cam_stereo_imp_->R2_.data)[i];

            for(int i=0; i<12; ++i)
                cam_info_r.P[i] = ((double*)capture_cam_stereo_imp_->P2_.data)[i];


//        rect_pub_l_.publish(img_msg_l, cam_info_l, header_l.stamp);
//        rect_pub_r_.publish(img_msg_r, cam_info_r, header_r.stamp);

            rect_it_pub_l_.publish(img_msg_l);
            rect_it_pub_r_.publish(img_msg_r);
            rect_caminfo_pub_l_.publish(cam_info_l);
            rect_caminfo_pub_r_.publish(cam_info_r);
        }

    private:
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MyApproxSyncStereoPolicy;
        message_filters::Synchronizer<MyApproxSyncStereoPolicy> * approx_sync_stereo_;

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MyExactSyncStereoPolicy;
        message_filters::Synchronizer<MyExactSyncStereoPolicy> * exact_sync_stereo_;

        image_transport::SubscriberFilter filter_img_l_;
        image_transport::SubscriberFilter filter_img_r_;

        image_transport::CameraPublisher rect_pub_l_;
        image_transport::CameraPublisher rect_pub_r_;

        image_transport::Publisher rect_it_pub_l_;
        image_transport::Publisher rect_it_pub_r_;
        ros::Publisher rect_caminfo_pub_l_;
        ros::Publisher rect_caminfo_pub_r_;

        CaptureCamStereoImp *capture_cam_stereo_imp_;
    };

    PLUGINLIB_EXPORT_CLASS(capture_cam::CaptureCamStereo, nodelet::Nodelet);
}


#endif // CAPTURE_CAM_STEREO_H_
