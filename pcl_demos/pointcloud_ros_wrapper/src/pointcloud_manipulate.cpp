#include "pointcloud_manipulate.h"

#include <chrono>

pcl::IndicesPtr PointcloudManipulate::Concatenate(const pcl::IndicesPtr & indicesA, const pcl::IndicesPtr & indicesB) {
    pcl::IndicesPtr ind(new std::vector<int>(*indicesA));
    ind->resize(ind->size() + indicesB->size());
    unsigned int oi = (unsigned int) indicesA->size();
    for (unsigned int i = 0; i < indicesB->size(); ++i)
        ind->at(oi++) = indicesB->at(i);
    return ind;
}

bool PointcloudManipulate::FilterByFOV () {

    if(cloud_rgb_input_ == NULL || cloud_rgb_output_ == NULL)
        return false;

    float fov_rad = max_fov_degree_ / 180.f * float(M_PI);

    size_t nSizeIn = cloud_rgb_input_->points.size();

    cloud_rgb_output_->points.reserve(nSizeIn);

    for(int i=0; i<nSizeIn; ++i) {
        float x = cloud_rgb_input_->points[i].x;
        float z = cloud_rgb_input_->points[i].z;
        float angle = atan2(x, z);
        float angleReal = std::abs(angle) * 2.f;
        if (angleReal < fov_rad)
            cloud_rgb_output_->points.push_back(cloud_rgb_input_->points[i]);
    }

    return true;
}

void PointcloudManipulate::TransformByImu () {

    Eigen::Vector3d v3_rpy_imu;
    tf2::Matrix3x3(
            tf2::Quaternion(quaternion_imu_.x, quaternion_imu_.y, quaternion_imu_.z, quaternion_imu_.w)).getRPY(
            v3_rpy_imu[0], v3_rpy_imu[1], v3_rpy_imu[2]);

    tf2::Quaternion tf2_q_imu;
    tf2_q_imu.setRPY(v3_rpy_imu[0], v3_rpy_imu[1], 0);
    tf2_q_imu.normalize();

    tf2::Matrix3x3(tf2_q_imu).getRPY(v3_rpy_imu[0], v3_rpy_imu[1], v3_rpy_imu[2]);
    v3_rpy_imu *= 180.0 * M_1_PI;
    ROS_INFO("imu Orientation r: [%f], p: [%f], y: [%f]", v3_rpy_imu[0], v3_rpy_imu[1], v3_rpy_imu[2]);

    Eigen::Quaternion<double> q_imu(tf2_q_imu.w(), tf2_q_imu.x(), tf2_q_imu.y(), tf2_q_imu.z());

    Eigen::Matrix4d m4_imu = Eigen::Matrix4d::Identity();
    m4_imu.block<3, 3>(0, 0) = q_imu.matrix();


    Eigen::Matrix4d m4_left_cloud;
    m4_left_cloud = Eigen::Matrix4d::Identity();
    m4_left_cloud(0, 3) = 0.06;


    Eigen::Matrix4d T_imu_cam;
    T_imu_cam <<
              0.00082475, -0.99999295, -0.00366264,  0.00326365,
              0.99999611,  0.00081498,  0.00266736, -0.04298796,
             -0.00266435, -0.00366482,  0.99998974,  0.02123103,
              0.0,         0.0,         0.0,         1.0;

    Eigen::Matrix4d m4_imu_left = T_imu_cam.inverse();


    Eigen::Matrix4d m4_init_imu   = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d m3_init_imu_y = Eigen::AngleAxis<double>(M_PI_2, Eigen::Vector3d(0, 1, 0)).toRotationMatrix();
    Eigen::Matrix3d m3_init_imu_z = Eigen::AngleAxis<double>(M_PI,   Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    m4_init_imu.block<3,3>(0,0) = m3_init_imu_y * m3_init_imu_z;


    Eigen::Matrix4d m4_cloud = m4_left_cloud * m4_imu_left * m4_init_imu * m4_imu *
                               m4_init_imu.inverse() * m4_imu_left.inverse() * m4_left_cloud.inverse();


    pcl::transformPointCloud(*cloud_rgb_input_, *cloud_rgb_output_, m4_cloud);
}

void PointcloudManipulate::SegmentForGround () {

    if(cloud_input_->empty()) {
        ROS_ERROR_STREAM("cloud_input_ is empty!");
        return;
    }


    auto now01 = std::chrono::high_resolution_clock::now();


    double cloud_angle_rad = -cam_angle_ / 180.0 * M_PI;
    Eigen::Matrix4d m4_transform = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d m3_rotation  = Eigen::AngleAxis<double>(cloud_angle_rad, Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
    m4_transform.block<3,3>(0,0) = m3_rotation;
    m4_transform(1,3) = -0.38;

    pcl::transformPointCloud(*cloud_input_, *cloud_input_, m4_transform);


    auto now02 = std::chrono::high_resolution_clock::now();
    ROS_DEBUG("pcl::transformPointCloud time_ms: %d", (unsigned int)(std::chrono::duration_cast<std::chrono::milliseconds>(now02 - now01).count()));


    pcl::CropBox<pcl::PointXYZ> filter_cropbox;
    filter_cropbox.setNegative(false);
    filter_cropbox.setMin(Eigen::Vector4f(-1.0, -0.05, 0, 1));
    filter_cropbox.setMax(Eigen::Vector4f(1.0, 0.05, 2, 1));
    filter_cropbox.setTransform(Eigen::Affine3f::Identity());
    filter_cropbox.setInputCloud(cloud_input_);
    filter_cropbox.filter(*cloud_input_cropbox_);
    if (cloud_input_cropbox_->empty()) {
        ROS_ERROR_STREAM("cloud_input_cropbox_ is empty!");
        return;
    }

    pcl::PassThrough<pcl::PointXYZ> filter_pass;
    filter_pass.setNegative(false);
    filter_pass.setFilterFieldName("z");
    filter_pass.setFilterLimits(roi_frontend_/100.0, roi_backend_/100.0);
    filter_pass.setInputCloud(cloud_input_cropbox_);
    filter_pass.filter(*cloud_input_pass_);
    if (cloud_input_pass_->empty()) {
        ROS_ERROR_STREAM("cloud_input_pass_ is empty!");
        return;
    }


    auto now04 = std::chrono::high_resolution_clock::now();
    ROS_DEBUG("filter_cropbox_pass time_ms: %d", (unsigned int)(std::chrono::duration_cast<std::chrono::milliseconds>(now04 - now02).count()));


    pcl::VoxelGrid<pcl::PointXYZ> filter_voxel;
    filter_voxel.setInputCloud(cloud_input_pass_);
    filter_voxel.setLeafSize(0.004f, 0.004f, 0.004f);
    filter_voxel.filter(*cloud_input_voxel_);


    cloud_input_ = cloud_input_voxel_;

    indices_cloud_in_->resize(cloud_input_->size());
    for(unsigned int i=0; i<cloud_input_->size(); ++i)
        indices_cloud_in_->at(i) = i;


    auto now05 = std::chrono::high_resolution_clock::now();
    ROS_DEBUG("filter_voxel time_ms: %d", (unsigned int)(std::chrono::duration_cast<std::chrono::milliseconds>(now05 - now04).count()));


    {
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;

        ne.setInputCloud (cloud_input_);
        ne.setIndices(indices_cloud_in_);

        if(indices_cloud_in_->size())
            kd_tree_->setInputCloud(cloud_input_, indices_cloud_in_);
        else
            kd_tree_->setInputCloud(cloud_input_);
        ne.setSearchMethod (kd_tree_);

        ne.setKSearch(5);

        Eigen::Vector3f viewpoint(0,0,0);
        if(viewpoint[0] != 0 || viewpoint[1] != 0 || viewpoint[2] != 0)
            ne.setViewPoint(viewpoint[0], viewpoint[1], viewpoint[2]);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        ne.compute (*cloud_normals);

        indices_flatsurfaces_->resize(cloud_normals->size());
        int oi = 0;
        for(unsigned int i=0; i<cloud_normals->size(); ++i) {
            Eigen::Vector4f v(cloud_normals->at(i).normal_x, cloud_normals->at(i).normal_y, cloud_normals->at(i).normal_z, 0.0f);
            double angle = pcl::getAngle3D(Eigen::Vector4f(0,1,0,0), v);
            if (angle < M_PI_4)
                indices_flatsurfaces_->at(oi++) = indices_cloud_in_->size() != 0 ? indices_cloud_in_->at(i) : i;
        }
        indices_flatsurfaces_->resize(oi);
        if(indices_flatsurfaces_->empty()) {
            ROS_ERROR_STREAM("indices_flatsurfaces_ is empty!");
            return;
        }
    }
    indices_cloud_in_ = indices_flatsurfaces_;


    auto now06 = std::chrono::high_resolution_clock::now();
    ROS_DEBUG("indices_flatsurfaces_ time_ms: %d", (unsigned int)(std::chrono::duration_cast<std::chrono::milliseconds>(now06 - now05).count()));


    int biggest_cluster_index = -1;
    {
        auto now061 = std::chrono::high_resolution_clock::now();

        std::vector<pcl::PointIndices> cluster_indices_;
        {
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster_extraction_;
            euclidean_cluster_extraction_.setClusterTolerance(0.05);
            euclidean_cluster_extraction_.setMinClusterSize(5);
            euclidean_cluster_extraction_.setMaxClusterSize(std::numeric_limits<int>::max());
            euclidean_cluster_extraction_.setInputCloud(cloud_input_);
            if (indices_cloud_in_->size()) {
                euclidean_cluster_extraction_.setIndices(indices_cloud_in_);
                kd_tree_->setInputCloud(cloud_input_, indices_cloud_in_);
            } else
                kd_tree_->setInputCloud(cloud_input_);
            euclidean_cluster_extraction_.setSearchMethod(kd_tree_);
            euclidean_cluster_extraction_.extract(cluster_indices_);
        }

        auto now062 = std::chrono::high_resolution_clock::now();
        ROS_DEBUG("06 pcl::EuclideanClusterExtraction time_ms: %d", (unsigned int)(std::chrono::duration_cast<std::chrono::milliseconds>(now062 - now061).count()));


        cluster_indices_flatsurfaces_.resize(cluster_indices_.size());

        int max_index = 0;
        size_t max_size = cluster_indices_[0].indices.size();
        for(unsigned int i=0; i<cluster_indices_.size(); ++i) {
            cluster_indices_flatsurfaces_[i] = pcl::IndicesPtr(new std::vector<int>(cluster_indices_[i].indices));
            if (max_size < cluster_indices_[i].indices.size()) {
                max_size = (unsigned int) cluster_indices_[i].indices.size();
                max_index = i;
            }
        }

        biggest_cluster_index = max_index;

        indices_ground_ = cluster_indices_flatsurfaces_[biggest_cluster_index];
        if(indices_ground_->empty()) {
            ROS_ERROR_STREAM("indices_ground_ is empty!");
            return;
        }
    }


    auto now07 = std::chrono::high_resolution_clock::now();
    ROS_DEBUG("indices_ground_ time_ms: %d", (unsigned int)(std::chrono::duration_cast<std::chrono::milliseconds>(now07 - now06).count()));


//    Eigen::Vector4f min,max;
//    pcl::getMinMax3D(*cloud_input_, *indices_ground_, min, max);
//
//    for(unsigned int i=0; i<cluster_indices_flatsurfaces_.size(); ++i) {
//        if ((int) i != biggest_cluster_index) {
//            Eigen::Vector4f centroid(0, 0, 0, 1);
//            pcl::compute3DCentroid(*cloud_input_, *cluster_indices_flatsurfaces_.at(i), centroid);
//            if (centroid[1] >= min[1] - 0.01 && centroid[1] <= max[1] + 0.01)
//                indices_ground_ = Concatenate(indices_ground_, cluster_indices_flatsurfaces_.at(i));
//        }
//    }


    cloud_ground_->resize(indices_ground_->size());
    for(unsigned int i=0; i<indices_ground_->size(); ++i)
        cloud_ground_->points[i] = cloud_input_->at(indices_ground_->at(i));


    pcl::copyPointCloud(*cloud_ground_, *cloud_output_);
}
