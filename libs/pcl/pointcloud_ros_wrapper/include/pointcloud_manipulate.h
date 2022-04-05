#ifndef PROJECT_POINTCLOUD_MANIPULATE_H
#define PROJECT_POINTCLOUD_MANIPULATE_H

//#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Geometry>

#include <tf_conversions/tf_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

const double PI_6_TAN = std::tan(M_PI / 6.0);

class PointcloudManipulate {

private:
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_cropbox_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_pass_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_voxel_;

    pcl::IndicesPtr indices_cloud_in_;

    pcl::IndicesPtr indices_flatsurfaces_;
    std::vector<pcl::IndicesPtr> cluster_indices_flatsurfaces_;

    pcl::IndicesPtr indices_ground_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground_;

public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_input_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_output_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output_;

    geometry_msgs::Quaternion quaternion_imu_;

    float max_fov_degree_;

    double cam_angle_;
    double roi_frontend_;
    double roi_backend_;

public:

    PointcloudManipulate() :
            kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>(false)),
            cloud_rgb_input_(new pcl::PointCloud<pcl::PointXYZRGB>),
            cloud_rgb_output_(new pcl::PointCloud<pcl::PointXYZRGB>),
            cloud_input_(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_output_(new pcl::PointCloud<pcl::PointXYZ>),
            max_fov_degree_(60),
            cam_angle_(30),
            roi_frontend_(10),
            roi_backend_(100) {

        cloud_input_cropbox_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        cloud_input_pass_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        cloud_input_voxel_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        indices_cloud_in_ = pcl::IndicesPtr(new std::vector<int>);

        indices_flatsurfaces_ = pcl::IndicesPtr(new std::vector<int>);

        indices_ground_ = pcl::IndicesPtr(new std::vector<int>);
        cloud_ground_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    ~PointcloudManipulate() {}

    pcl::IndicesPtr Concatenate (const pcl::IndicesPtr & indicesA, const pcl::IndicesPtr & indicesB);

    bool FilterByFOV ();

    void TransformByImu ();

    void SegmentForGround ();
};

#endif //PROJECT_POINTCLOUD_MANIPULATE_H
