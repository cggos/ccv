#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace capture_image
{

    class CaptureImage : public nodelet::Nodelet
    {
        public:
            CaptureImage() 
                : approxSyncStereo_(0)
                , exactSyncStereo_(0) 
                , num_corner_h_(0)
                , num_corner_v_(0)
                , str_imgs_dir("./") {}

            virtual ~CaptureImage()
            {
                if(approxSyncStereo_)
                    delete approxSyncStereo_;
                if(exactSyncStereo_)
                    delete exactSyncStereo_;
            }

        private:
            virtual void onInit()
            {
                ros::NodeHandle & nh = getNodeHandle();
                ros::NodeHandle & pnh = getPrivateNodeHandle();

                int queueSize = 10;
                n_image_pair = 0;
                bool approxSync = true;
                pnh.param("approx_sync", approxSync, approxSync);
                pnh.param("imgs_dir", str_imgs_dir, str_imgs_dir);
                pnh.param("num_corner_h", num_corner_h_, 7);
                pnh.param("num_corner_v", num_corner_v_, 6);

                if(approxSync)
                {

                    approxSyncStereo_ = new message_filters::Synchronizer<MyApproxSyncStereoPolicy>(MyApproxSyncStereoPolicy(queueSize), imageLeft_, imageRight_, cameraInfoLeft_, cameraInfoRight_);
                    approxSyncStereo_->registerCallback(boost::bind(&CaptureImage::stereoCallback, this, _1, _2, _3, _4));
                }
                else
                {
                    exactSyncStereo_ = new message_filters::Synchronizer<MyExactSyncStereoPolicy>(MyExactSyncStereoPolicy(queueSize), imageLeft_, imageRight_, cameraInfoLeft_, cameraInfoRight_);
                    exactSyncStereo_->registerCallback(boost::bind(&CaptureImage::stereoCallback, this, _1, _2, _3, _4));
                }

                ros::NodeHandle left_nh(nh, "left");
                ros::NodeHandle right_nh(nh, "right");
                ros::NodeHandle left_pnh(pnh, "left");
                ros::NodeHandle right_pnh(pnh, "right");
                image_transport::ImageTransport left_it(left_nh);
                image_transport::ImageTransport right_it(right_nh);
                image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), left_pnh);
                image_transport::TransportHints hintsRight("raw", ros::TransportHints(), right_pnh);

                imageLeft_.subscribe(left_it, left_nh.resolveName("image"), 1, hintsLeft);
                imageRight_.subscribe(right_it, right_nh.resolveName("image"), 1, hintsRight);
                cameraInfoLeft_.subscribe(left_nh, "camera_info", 1);
                cameraInfoRight_.subscribe(right_nh, "camera_info", 1);
            }

            void stereoCallback(const sensor_msgs::ImageConstPtr& imageLeft,
                    const sensor_msgs::ImageConstPtr& imageRight,
                    const sensor_msgs::CameraInfoConstPtr& camInfoLeft,
                    const sensor_msgs::CameraInfoConstPtr& camInfoRight)
            {
                if(!(imageLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
                            imageLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
                            imageLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
                            imageLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
                        !(imageRight->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
                            imageRight->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
                            imageRight->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
                            imageRight->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
                {
                    NODELET_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (enc=%s)", imageLeft->encoding.c_str());
                    return;
                }


                cv_bridge::CvImageConstPtr ptrLeftImage, ptrRightImage;
                ptrLeftImage  = cv_bridge::toCvShare(imageLeft, "mono8");
                ptrRightImage = cv_bridge::toCvShare(imageRight, "mono8");

                cv::Mat mat_left  = ptrLeftImage->image;
                cv::Mat mat_right = ptrRightImage->image;

                int numCornersHor = 11;
                int numCornersVer = 7;
                cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);

                cv::Mat mat_left_color(mat_left.size(), CV_8UC3);
                cv::Mat mat_right_color(mat_right.size(), CV_8UC3);

                cv::cvtColor(mat_left,  mat_left_color,  CV_GRAY2BGR);
                cv::cvtColor(mat_right, mat_right_color, CV_GRAY2BGR);

                std::vector<cv::Point2f> corners_l;
                std::vector<cv::Point2f> corners_r;
                bool found_l = cv::findChessboardCorners(mat_left,  board_sz, corners_l, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
                bool found_r = cv::findChessboardCorners(mat_right, board_sz, corners_r, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
                if(found_l && found_r)
                {
                    cv::cornerSubPix(mat_left,  corners_l, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                    cv::cornerSubPix(mat_right, corners_r, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

                    cv::drawChessboardCorners(mat_left_color,  board_sz, corners_l, found_l);
                    cv::drawChessboardCorners(mat_right_color, board_sz, corners_r, found_r);

                    char key = (char)cv::waitKey(10);
                    if(key == 'c') {
                        std::ostringstream oss_left;
                        std::ostringstream oss_right;
                        std::string left  = str_imgs_dir + "/left";
                        std::string right = str_imgs_dir + "/right";
                        oss_left  << left  << n_image_pair << ".jpg";
                        oss_right << right << n_image_pair << ".jpg";

                        cv::imwrite(oss_left.str(),  mat_left);
                        cv::imwrite(oss_right.str(), mat_right);

                        n_image_pair++;

                        ROS_INFO("found and save image pair %d", n_image_pair);
                    }
                }

                cv::Mat mat_all;
                cv::hconcat(mat_left_color, mat_right_color, mat_all);
                cv::imshow("left and right color image", mat_all);

                cv::waitKey(10);
            }

        private:

            int n_image_pair ;
            int num_corner_h_;
            int num_corner_v_;

            std::string str_imgs_dir;

            image_transport::SubscriberFilter imageLeft_;
            image_transport::SubscriberFilter imageRight_;
            message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
            message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;

            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyApproxSyncStereoPolicy;
            message_filters::Synchronizer<MyApproxSyncStereoPolicy> * approxSyncStereo_;

            typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyExactSyncStereoPolicy;
            message_filters::Synchronizer<MyExactSyncStereoPolicy> * exactSyncStereo_;

    };

    PLUGINLIB_EXPORT_CLASS(capture_image::CaptureImage, nodelet::Nodelet);
}

