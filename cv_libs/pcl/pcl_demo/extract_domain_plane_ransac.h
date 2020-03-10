#ifndef EXTRACT_DOMAIN_PLANE_RANSAC_H
#define EXTRACT_DOMAIN_PLANE_RANSAC_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

PointCloudXYZ::Ptr extract_domain_plane_ransac(const PointCloudXYZ::Ptr cloud_in, float thr = 0.05);

void calc_plane_normal(const PointCloudXYZ::Ptr cloud_in, Eigen::Vector3f &v3normal, Eigen::Vector3f &v3centroid);

Eigen::Affine3f get_transform_new_from_old(const Eigen::Vector3f &v3normal, const Eigen::Vector3f &v3centroid);

#endif // EXTRACT_DOMAIN_PLANE_RANSAC_H