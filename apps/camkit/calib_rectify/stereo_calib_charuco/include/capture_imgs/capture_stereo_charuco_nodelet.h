#ifndef CAPTURE_STEREO_CHARUCO_H_
#define CAPTURE_STEREO_CHARUCO_H_

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

#include "capture_imgs/detect_charuco.h"

#include "capture_imgs/coc_stereo_calib.h"
#include "capture_imgs/coc_stereo_rectify.h"

namespace capture_imgs {

    class CaptureStereoCharuco : public nodelet::Nodelet {
    public:
        CaptureStereoCharuco() : is_rectify_(false) {}

        virtual ~CaptureStereoCharuco() {
            if(approx_sync_stereo_)
                delete approx_sync_stereo_;
            if(exact_sync_stereo_)
                delete exact_sync_stereo_;
        }

    private:
        virtual void onInit() {
            ros::NodeHandle &nh  = getNodeHandle();
            ros::NodeHandle &pnh = getPrivateNodeHandle();

            pnh.param("is_rectify", is_rectify_, is_rectify_);

            bool approx_sync = true;
            pnh.param("approx_sync", approx_sync, approx_sync);

            int queue_size = 10;
            if(approx_sync) {
                approx_sync_stereo_ = new message_filters::Synchronizer<MyApproxSyncStereoPolicy>(
                        MyApproxSyncStereoPolicy(queue_size), filter_img_l_, filter_img_r_);
                approx_sync_stereo_->registerCallback(
                        boost::bind(&CaptureStereoCharuco::stereoCallback, this, _1, _2));
            }
            else {
                exact_sync_stereo_ = new message_filters::Synchronizer<MyExactSyncStereoPolicy>(
                        MyExactSyncStereoPolicy(queue_size), filter_img_l_, filter_img_r_);
                exact_sync_stereo_->registerCallback(
                        boost::bind(&CaptureStereoCharuco::stereoCallback, this, _1, _2));
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
//            rect_pub_l_ = it_l.advertiseCamera("image", 1, false);
//            rect_pub_r_ = it_r.advertiseCamera("image", 1, false);

            rect_it_pub_l_      = it_l.advertise("image", 1);
            rect_it_pub_r_      = it_r.advertise("image", 1);
            rect_caminfo_pub_l_ = nh_rect_l.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
            rect_caminfo_pub_r_ = nh_rect_r.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

            int id_dictionary = 6;
            int squares_x;
            int squares_y;
            float square_length;
            float marker_length;
            pnh.param("squares_x", squares_x, 7);
            pnh.param("squares_y", squares_y, 11);
            pnh.param("square_length", square_length, 0.021f);
            pnh.param("marker_length", marker_length, 0.014f);
            charuco_detector_r_ = charuco_detector_l_ =
                    CharucoDetector(id_dictionary, squares_x, squares_y, square_length, marker_length);

            std::string str_param_dir = "";
            pnh.param("param_dir", str_param_dir, str_param_dir);
            coc_stereo_calib_ = CocStereoCalib(cv::Size(squares_x-1, squares_y-1), str_param_dir);

            coc_stereo_rectify_ = CocStereoRectify(str_param_dir);
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

            process(img_l, img_r);

            if(is_rectify_)
                pub_img_rect(img_rect_l_, img_rect_r_, image_left->header, image_right->header);
        }

        void process(const cv::Mat &img_l, const cv::Mat &img_r);

        void calib(const cv::Mat &img_l, const cv::Mat &img_r);
        void rectify(const cv::Mat &img_l, const cv::Mat &img_r);

        void pub_img_rect(
                const cv::Mat &img_l, const cv::Mat &img_r,
                const std_msgs::Header &header_l,
                const std_msgs::Header &header_r);

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

        CharucoDetector charuco_detector_l_;
        CharucoDetector charuco_detector_r_;

        CocStereoCalib coc_stereo_calib_;
        CocStereoRectify coc_stereo_rectify_;

        bool is_rectify_;

        cv::Mat img_rect_l_;
        cv::Mat img_rect_r_;
    };

    PLUGINLIB_EXPORT_CLASS(capture_imgs::CaptureStereoCharuco, nodelet::Nodelet);
}


#endif // CAPTURE_STEREO_CHARUCO_H_
