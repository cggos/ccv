#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "cgocv/stereo_camera.h"

namespace stereo_reconstruct {

    class StereoReconstruct : public nodelet::Nodelet {
    public:
        StereoReconstruct() :
                max_depth_(0.0),
                min_depth_(0.0),
                voxel_size_(0.0),
                approx_sync_stereo_(0),
                exact_sync_stereo_(0),
                is_mm_(true),
                is_cloudfilter_(true),
                is_use_colormap_(false),
                frame_id_depth_("stereo_depth_optical_frame"),
                frame_id_cloud_("stereo_cloud_optical_frame"),
                pcl_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
                depth_frame_(nullptr) {}

        virtual ~StereoReconstruct() {
            if (approx_sync_stereo_)
                delete approx_sync_stereo_;
            if (exact_sync_stereo_)
                delete exact_sync_stereo_;
            if (depth_frame_)
                delete depth_frame_;
        }

    private:
        virtual void onInit() {
            ros::NodeHandle &nh  = getNodeHandle();
            ros::NodeHandle &pnh = getPrivateNodeHandle();

            bool approx_sync = true;

            pnh.param("approx_sync", approx_sync, approx_sync);
            pnh.param("max_depth",  max_depth_, max_depth_);
            pnh.param("min_depth",  min_depth_, min_depth_);
            pnh.param("voxel_size", voxel_size_, voxel_size_);
            pnh.param("is_mm", is_mm_, is_mm_);
            pnh.param("is_cloudfilter", is_cloudfilter_, is_cloudfilter_);
            pnh.param("is_use_colormap", is_use_colormap_, is_use_colormap_);
            pnh.param("frame_id_cloud", frame_id_cloud_, frame_id_cloud_);
            pnh.param("frame_id_depth", frame_id_depth_, frame_id_depth_);

            NODELET_INFO("Approximate time sync = %s", approx_sync ? "true" : "false");

            if (approx_sync) {
                approx_sync_stereo_ = new message_filters::Synchronizer<MyApproxSyncStereoPolicy>(
                        MyApproxSyncStereoPolicy(10), image_left_, image_right_, camera_info_left_,
                        camera_info_right_);
                approx_sync_stereo_->registerCallback(
                        boost::bind(&StereoReconstruct::stereoCallback, this, _1, _2, _3, _4));
            } else {
                exact_sync_stereo_ = new message_filters::Synchronizer<MyExactSyncStereoPolicy>(
                        MyExactSyncStereoPolicy(10), image_left_, image_right_, camera_info_left_, camera_info_right_);
                exact_sync_stereo_->registerCallback(
                        boost::bind(&StereoReconstruct::stereoCallback, this, _1, _2, _3, _4));
            }

            ros::NodeHandle left_nh(nh, "left");
            ros::NodeHandle right_nh(nh, "right");
            ros::NodeHandle left_pnh(pnh, "left");
            ros::NodeHandle right_pnh(pnh, "right");
            image_transport::ImageTransport left_it(left_nh);
            image_transport::ImageTransport right_it(right_nh);
            image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), left_pnh);
            image_transport::TransportHints hintsRight("raw", ros::TransportHints(), right_pnh);

            image_left_.subscribe(left_it, left_nh.resolveName("image"), 1, hintsLeft);
            image_right_.subscribe(right_it, right_nh.resolveName("image"), 1, hintsRight);
            camera_info_left_.subscribe(left_nh, "camera_info", 1);
            camera_info_right_.subscribe(right_nh, "camera_info", 1);

            cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

            image_transport::ImageTransport depth_it(nh);
            depth_pub_ = depth_it.advertiseCamera("depth", 1, false);
        }

        void stereoCallback(const sensor_msgs::ImageConstPtr &image_left,
                            const sensor_msgs::ImageConstPtr &image_right,
                            const sensor_msgs::CameraInfoConstPtr &cam_info_left,
                            const sensor_msgs::CameraInfoConstPtr &cam_info_right) {

            if (!(image_left->encoding.compare(sensor_msgs::image_encodings::MONO8)  == 0
                  || image_left->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0
                  || image_left->encoding.compare(sensor_msgs::image_encodings::BGR8)   == 0
                  || image_left->encoding.compare(sensor_msgs::image_encodings::RGB8)   == 0)
                ||
                !(image_right->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0
                  || image_right->encoding.compare(sensor_msgs::image_encodings::MONO16)== 0
                  || image_right->encoding.compare(sensor_msgs::image_encodings::BGR8)  == 0
                  || image_right->encoding.compare(sensor_msgs::image_encodings::RGB8)  == 0)) {
                NODELET_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (enc=%s)", image_left->encoding.c_str());
                return;
            }

            if (cloud_pub_.getNumSubscribers() || depth_pub_.getNumSubscribers()) {

                cv_bridge::CvImageConstPtr ptrLeftImage  = cv_bridge::toCvShare(image_left,  "mono8");
                cv_bridge::CvImageConstPtr ptrRightImage = cv_bridge::toCvShare(image_right, "mono8");

                const cv::Mat &mat_left  = ptrLeftImage->image;
                const cv::Mat &mat_right = ptrRightImage->image;

                image_geometry::StereoCameraModel stereo_camera_model;
                stereo_camera_model.fromCameraInfo(*cam_info_left, *cam_info_right);

                stereo_camera_.camera_model_.baseline = stereo_camera_model.baseline();
                stereo_camera_.camera_model_.left.cx  = stereo_camera_model.left().cx();
                stereo_camera_.camera_model_.left.cy  = stereo_camera_model.left().cy();
                stereo_camera_.camera_model_.left.fx  = stereo_camera_model.left().fx();
                stereo_camera_.camera_model_.right.cx = stereo_camera_model.right().cx();
                stereo_camera_.camera_model_.right.fx = stereo_camera_model.right().fx();

                cv::Mat mat_disp;
                stereo_camera_.compute_disparity_map(mat_left, mat_right, mat_disp);

                if (depth_frame_ == nullptr)
                    depth_frame_ = new cv::Mat(mat_disp.size(), is_mm_ ? CV_16UC1 : CV_32FC1);
                stereo_camera_.disparity_to_depth_map(mat_disp, *depth_frame_);

                stereo_camera_.depth_to_pointcloud(*depth_frame_, mat_left, *pcl_cloud_);

                if(is_use_colormap_)
                {
                    cv::Mat colormap;
                    stereo_camera_.generate_colormap(*depth_frame_, &colormap);
                    cv::imshow("depth colormap", colormap);
                    cv::waitKey(3);
                }

                publishDepth(*depth_frame_, cam_info_left);

                publishCloud(pcl_cloud_, image_left->header);
            }
        }

        void publishDepth(cv::Mat &depth, const sensor_msgs::CameraInfoConstPtr &cam_info) {

            std::string encoding = "";
            switch (depth.type()) {
                case CV_16UC1:
                    encoding = sensor_msgs::image_encodings::TYPE_16UC1;
                    break;
                case CV_32FC1:
                    encoding = sensor_msgs::image_encodings::TYPE_32FC1;
                    break;
            }

            sensor_msgs::Image depth_msg;
            std_msgs::Header depth_header;
            depth_header.frame_id = frame_id_depth_;
            depth_header.stamp    = ros::Time::now();
            cv_bridge::CvImage(depth_header, encoding, depth).toImageMsg(depth_msg);

            sensor_msgs::CameraInfo depth_info;
            depth_info = *cam_info;
            depth_info.header = depth_msg.header;

            depth_pub_.publish(depth_msg, depth_info, ros::Time::now());
        }

        void publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, const std_msgs::Header &header) {

            if(is_cloudfilter_) {

                if (pcl_cloud->size() && (min_depth_ != 0.0 || max_depth_ > min_depth_)) {
                    pcl::PassThrough<pcl::PointXYZRGB> filter;
                    filter.setNegative(false);
                    filter.setFilterFieldName("z");
                    filter.setFilterLimits(min_depth_,
                                           max_depth_ > min_depth_ ? max_depth_ : std::numeric_limits<float>::max());
                    filter.setInputCloud(pcl_cloud);
                    filter.filter(*pcl_cloud);
                }

                if (pcl_cloud->size() && voxel_size_ > 0.0) {
                    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
                    filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
                    filter.setInputCloud(pcl_cloud);
                    filter.filter(*pcl_cloud);
                }

                double noise_filter_radius = 0.0;
                int noise_filter_min_neighbors = 5;
                if (pcl_cloud->empty() && noise_filter_radius > 0.0 && noise_filter_min_neighbors > 0) {
                    if (voxel_size_ <= 0.0 && !(min_depth_ != 0.0 || max_depth_ > min_depth_)) {
                        std::vector<int> indices;
                        pcl::removeNaNFromPointCloud(*pcl_cloud, *pcl_cloud, indices);
                    }

                    pcl::IndicesPtr indices(new std::vector<int>(pcl_cloud->size()));
                    {
                        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
                                new pcl::search::KdTree<pcl::PointXYZRGB>(false));
                        int oi = 0;
                        tree->setInputCloud(pcl_cloud);
                        for (unsigned int i = 0; i < pcl_cloud->size(); ++i) {
                            std::vector<int> kIndices;
                            std::vector<float> kDistances;
                            int k = tree->radiusSearch(pcl_cloud->at(i), noise_filter_radius, kIndices, kDistances);
                            if (k > noise_filter_min_neighbors)
                                indices->at(oi++) = i;

                        }
                        indices->resize(oi);
                    }

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::copyPointCloud(*pcl_cloud, *indices, *tmp);
                    pcl_cloud = tmp;
                }
            }

            sensor_msgs::PointCloud2 ros_cloud;
            pcl::toROSMsg(*pcl_cloud, ros_cloud);
            ros_cloud.header.stamp = header.stamp;
            ros_cloud.header.frame_id = frame_id_cloud_;

            cloud_pub_.publish(ros_cloud);
        }

    private:
        double max_depth_;
        double min_depth_;
        double voxel_size_;
        bool is_mm_;
        bool is_cloudfilter_;
        bool is_use_colormap_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_;
        cv::Mat *depth_frame_;

        ros::Publisher cloud_pub_;
        image_transport::CameraPublisher depth_pub_;

        image_transport::SubscriberFilter image_left_;
        image_transport::SubscriberFilter image_right_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_left_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_right_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyApproxSyncStereoPolicy;
        message_filters::Synchronizer<MyApproxSyncStereoPolicy> *approx_sync_stereo_;

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyExactSyncStereoPolicy;
        message_filters::Synchronizer<MyExactSyncStereoPolicy> *exact_sync_stereo_;

        std::string frame_id_cloud_;
        std::string frame_id_depth_;

        cg::StereoCamera stereo_camera_;
    };

    PLUGINLIB_EXPORT_CLASS(stereo_reconstruct::StereoReconstruct, nodelet::Nodelet);
}
