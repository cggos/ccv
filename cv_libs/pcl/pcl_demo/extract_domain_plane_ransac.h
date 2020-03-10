#ifndef EXTRACT_DOMAIN_PLANE_RANSAC_H
#define EXTRACT_DOMAIN_PLANE_RANSAC_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

PointCloudXYZ::Ptr extract_domain_plane_ransac(const PointCloudXYZ::Ptr cloud_in, float thr = 0.05);

void calc_plane_normal(const PointCloudXYZ::Ptr cloud_in, pcl::Normal &normal);

#endif // EXTRACT_DOMAIN_PLANE_RANSAC_H