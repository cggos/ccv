#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
cv::Mat rmap[2][2];
cv::Mat r_img1, r_img2;
image_transport::Publisher rectified_l_publisher, rectified_r_publisher;
std::string stereo_parameters;
int IMG_HEIGHT_R;
int IMG_WIDTH_R;



void fillCamInfo(
        const sensor_msgs::CameraInfoPtr &left_cam_info_msg,
        const sensor_msgs::CameraInfoPtr &right_cam_info_msg,
        const std::string &left_frame_id,
        const std::string &right_frame_id,cv::Vec4d& D1,cv::Vec4d& D2,cv::Mat& K1,cv::Mat& K2,cv::Mat& R1,cv::Mat& R2,cv::Mat& R,cv::Mat& P1, cv::Mat& P2, cv::Vec3d& T) {

    left_cam_info_msg->distortion_model =
        sensor_msgs::distortion_models::PLUMB_BOB;
    right_cam_info_msg->distortion_model =
        sensor_msgs::distortion_models::PLUMB_BOB;

    left_cam_info_msg->D.resize(5);
    right_cam_info_msg->D.resize(5);
    left_cam_info_msg->D[0] = D1[0];   // k1
    left_cam_info_msg->D[1] = D1[1];   // k2
    left_cam_info_msg->D[2] = D1[2];   // k3
    left_cam_info_msg->D[3] = D1[3];   // p1
    left_cam_info_msg->D[4] = 0;   // p2
    right_cam_info_msg->D[0] = D2[0]; // k1
    right_cam_info_msg->D[1] = D2[1]; // k2
    right_cam_info_msg->D[2] = D2[2]; // k3
    right_cam_info_msg->D[3] = D2[3]; // p1
    right_cam_info_msg->D[4] = 0; // p2

    left_cam_info_msg->K.fill(0.0);
    right_cam_info_msg->K.fill(0.0);
    left_cam_info_msg->K[0] = K1.at<double>(0,0);
    left_cam_info_msg->K[2] = K1.at<double>(0,2);
    left_cam_info_msg->K[4] = K1.at<double>(1,1);
    left_cam_info_msg->K[5] = K1.at<double>(1,2);
    left_cam_info_msg->K[8] = 1.0;
    right_cam_info_msg->K[0] = K2.at<double>(0,0);;
    right_cam_info_msg->K[2] = K2.at<double>(0,2);;
    right_cam_info_msg->K[4] = K2.at<double>(1,1);
    right_cam_info_msg->K[5] = K2.at<double>(1,2);
    right_cam_info_msg->K[8] = 1.0;
    left_cam_info_msg->R.fill(0.0);
    right_cam_info_msg->R.fill(0.0);

    left_cam_info_msg->R[0] = R1.at<double>(0,0);
    left_cam_info_msg->R[1] = R1.at<double>(0,1);
    left_cam_info_msg->R[2] = R1.at<double>(0,2);
    left_cam_info_msg->R[3] = R1.at<double>(1,0);
    left_cam_info_msg->R[4] = R1.at<double>(1,1);
    left_cam_info_msg->R[5] = R1.at<double>(1,2);
    left_cam_info_msg->R[6] = R1.at<double>(2,0);
    left_cam_info_msg->R[7] = R1.at<double>(2,1);
    left_cam_info_msg->R[8] = R1.at<double>(2,2);

    right_cam_info_msg->R[0] = R2.at<double>(0,0);
    right_cam_info_msg->R[1] = R2.at<double>(0,1);
    right_cam_info_msg->R[2] = R2.at<double>(0,2);
    right_cam_info_msg->R[3] = R2.at<double>(1,0);
    right_cam_info_msg->R[4] = R2.at<double>(1,1);
    right_cam_info_msg->R[5] = R2.at<double>(1,2);
    right_cam_info_msg->R[6] = R2.at<double>(2,0);
    right_cam_info_msg->R[7] = R2.at<double>(2,1);
    right_cam_info_msg->R[8] = R2.at<double>(2,2);

    left_cam_info_msg->P.fill(0.0);
    right_cam_info_msg->P.fill(0.0);

    left_cam_info_msg->P[0] = P1.at<double>(0,0);
    left_cam_info_msg->P[1] = P1.at<double>(0,1);
    left_cam_info_msg->P[2] = P1.at<double>(0,2);
    left_cam_info_msg->P[3] = P1.at<double>(0,3);
    left_cam_info_msg->P[4] = P1.at<double>(1,0);
    left_cam_info_msg->P[5] = P1.at<double>(1,1);
    left_cam_info_msg->P[6] = P1.at<double>(1,2);
    left_cam_info_msg->P[7] = P1.at<double>(1,3);
    left_cam_info_msg->P[8] = P1.at<double>(2,0);
    left_cam_info_msg->P[9] = P1.at<double>(2,1);
    left_cam_info_msg->P[10] = P1.at<double>(2,2);
    left_cam_info_msg->P[11] = P1.at<double>(2,3);

    right_cam_info_msg->P[0] = P2.at<double>(0,0);
    right_cam_info_msg->P[1] = P2.at<double>(0,1);
    right_cam_info_msg->P[2] = P2.at<double>(0,2);
    right_cam_info_msg->P[3] = P2.at<double>(0,3);
    right_cam_info_msg->P[4] = P2.at<double>(1,0);
    right_cam_info_msg->P[5] = P2.at<double>(1,1);
    right_cam_info_msg->P[6] = P2.at<double>(1,2);
    right_cam_info_msg->P[7] = P2.at<double>(1,3);
    right_cam_info_msg->P[8] = P2.at<double>(2,0);
    right_cam_info_msg->P[9] = P2.at<double>(2,1);
    right_cam_info_msg->P[10] = P2.at<double>(2,2);
    right_cam_info_msg->P[11] = P2.at<double>(2,3);


    left_cam_info_msg->width = right_cam_info_msg->width = IMG_WIDTH_R;
    left_cam_info_msg->height = right_cam_info_msg->height = IMG_HEIGHT_R;

    left_cam_info_msg->header.frame_id = left_frame_id;
    right_cam_info_msg->header.frame_id = right_frame_id;
}

void publishCamInfo(const sensor_msgs::CameraInfoPtr &cam_info_msg,
        const ros::Publisher &pub_cam_info,
        const ros::Time &stamp) {
    static int seq = 0;
    cam_info_msg->header.stamp = stamp;
    cam_info_msg->header.seq = seq;
    pub_cam_info.publish(cam_info_msg);
    //ROS_INFO_STREAM("==== " << cam_info_msg->width);
    ++seq;
}

void readCalibrationParameters(cv::Mat &K1, cv::Mat &K2, cv::Mat &R,
        cv::Mat &R1, cv::Mat &R2, cv::Mat &P1,
        cv::Mat &P2, cv::Vec3d &T,
        cv::Vec4d &D1, cv::Vec4d &D2, cv::Size &img_size)
{
    cv::FileStorage fs(stereo_parameters,cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("Failed to open calibration parameter file.");
        exit(1);
    }
    fs["K1"] >> K1;
    fs["K2"] >> K2;
    fs["R"] >> R;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["T"] >> T;
    fs["D1"] >> D1;
    fs["D2"] >> D2;
    img_size.height = IMG_HEIGHT_R;
    img_size.width = IMG_WIDTH_R;
}

void applyRectificationMaps(cv::Mat img_left, cv::Mat img_right)
{
    cv::remap(img_left, r_img1, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
    cv::remap(img_right, r_img2, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
    if(r_img1.channels() > 1)
        cv::cvtColor(r_img1,r_img1,CV_RGB2GRAY);
    if(r_img2.channels() > 1)
        cv::cvtColor(r_img2,r_img2,CV_RGB2GRAY);
}

void publishRectifiedImages(cv::Mat &left_rect, cv::Mat &right_rect, cv_bridge::CvImagePtr cv_ptr)
{
    cv_bridge::CvImage out_rect_l;
    cv_bridge::CvImage out_rect_r;
    out_rect_l.header 	= cv_ptr->header;
    out_rect_r.header	= cv_ptr->header;
    out_rect_l.encoding 	= sensor_msgs::image_encodings::MONO8;
    out_rect_r.encoding 	= sensor_msgs::image_encodings::MONO8;
    cv::Mat r_l_u8, r_r_u8;
    left_rect.convertTo(r_l_u8, CV_8UC1);
    right_rect.convertTo(r_r_u8, CV_8UC2);
    out_rect_l.image = r_l_u8;
    out_rect_r.image = r_r_u8;
    rectified_l_publisher.publish(out_rect_l.toImageMsg());
    rectified_r_publisher.publish(out_rect_r.toImageMsg());


}

void rectifyCallback(const sensor_msgs::ImageConstPtr& cam_l_msg, const sensor_msgs::ImageConstPtr& cam_r_msg,sensor_msgs::CameraInfoPtr left_cam_info_msg,sensor_msgs::CameraInfoPtr right_cam_info_msg,ros::Publisher pub_left_cam_info,ros::Publisher pub_right_cam_info)
{
    cv_bridge::CvImagePtr cv_ptr_1, cv_ptr_2;
    cv::Mat img_1, img_2;
    try
    {
        cv_ptr_1 = cv_bridge::toCvCopy(cam_l_msg, sensor_msgs::image_encodings::MONO8);
        cv_ptr_2 = cv_bridge::toCvCopy(cam_r_msg, sensor_msgs::image_encodings::MONO8);
        cv::Mat img1 = cv_ptr_1->image;
        cv::Mat img2 = cv_ptr_2->image;
        applyRectificationMaps(img1, img2);
        publishRectifiedImages(r_img1, r_img2, cv_ptr_1);
        publishCamInfo(left_cam_info_msg, pub_left_cam_info, ros::Time::now());
        publishCamInfo(right_cam_info_msg, pub_right_cam_info, ros::Time::now());
        //ROS_INFO("Rectified image pair..");
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not execute callback function for stereo rectify");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_rectify");
    ros::NodeHandle nh("~");

    std::string left_frame_id  = "/mynt_left_frame";
    std::string right_frame_id = "/mynt_right_frame";;

    nh.param("calib_file", stereo_parameters, std::string(" "));

    nh.param("img_width",  IMG_WIDTH_R,  1280);
    nh.param("img_height", IMG_HEIGHT_R, 1024);

    nh.param("left_frame_id",  left_frame_id,  std::string(""));
    nh.param("right_frame_id", right_frame_id, std::string(""));

    ROS_INFO("Calibration file location (Stereo Fisheye Rectify): %s", stereo_parameters.c_str());

    cv::Vec3d T;
    cv::Vec4d D1,D2;
    cv::Mat R1, R2, P1, P2, K1, K2, R;
    cv::Size img_size;

    readCalibrationParameters(K1, K2, R, R1, R2, P1, P2, T, D1, D2, img_size);

    cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, img_size, CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::fisheye::initUndistortRectifyMap(K2, D2, R2, P2, img_size, CV_16SC2, rmap[1][0], rmap[1][1]);

    sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr right_cam_info_msg(new sensor_msgs::CameraInfo());

    
    fillCamInfo( left_cam_info_msg, right_cam_info_msg, left_frame_id, right_frame_id,D1, D2,K1,K2,R1, R2, R, P1, P2,  T);

    ros::Publisher pub_left_cam_info;
    ros::Publisher pub_right_cam_info;

    std::string left_cam_info_topic = "left/camera_info";
    std::string right_cam_info_topic = "right/camera_info";

    image_transport::ImageTransport it(nh);

    message_filters::Subscriber<sensor_msgs::Image> image_sub_1(nh, "left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_2(nh, "right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), image_sub_1, image_sub_2);
    pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic,1);
    pub_right_cam_info = nh.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic,1);
    rectified_l_publisher	= it.advertise("left/rectified",1);
    rectified_r_publisher 	= it.advertise("right/rectified",1);
    sync.registerCallback(boost::bind(&rectifyCallback, _1, _2,left_cam_info_msg,right_cam_info_msg,pub_left_cam_info,pub_right_cam_info));


    ros::spin();
}
