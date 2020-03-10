#include "extract_domain_plane_ransac.h"

#include <Eigen/Eigenvalues>

PointCloudXYZ::Ptr extract_domain_plane_ransac(const PointCloudXYZ::Ptr cloud_in, float thr) {
    PointCloudXYZ::Ptr cloud_out(new PointCloudXYZ);

    size_t nPoints = cloud_in->size();
    Eigen::Vector3f v3BestMean;
    Eigen::Vector3f v3BestNormal;
    double dBestDistSquared = 9999999999999999.9;
    int nRansacs = 100;
    for(int i=0; i<nRansacs; i++) {
        int nA = rand() % nPoints;
        int nB = nA;
        int nC = nA;
        while (nB == nA)
            nB = rand() % nPoints;
        while (nC == nA || nC == nB)
            nC = rand() % nPoints;
        
        Eigen::Vector3f v3A = cloud_in->points[nA].getVector3fMap();
        Eigen::Vector3f v3B = cloud_in->points[nB].getVector3fMap();
        Eigen::Vector3f v3C = cloud_in->points[nC].getVector3fMap();

        Eigen::Vector3f v3Mean = 0.33333333 * (v3A + v3B + v3C);

        Eigen::Vector3f v3CA = v3C - v3A;
        Eigen::Vector3f v3BA = v3B - v3A;
        Eigen::Vector3f v3Normal = v3CA.cross(v3BA);

        if (v3Normal.dot(v3Normal) == 0)
            continue;

        v3Normal.normalize();

        double dSumError = 0.0;
        for (unsigned int i = 0; i < nPoints; i++) {
            Eigen::Vector3f v3i = cloud_in->points[i].getVector3fMap();
            Eigen::Vector3f v3Diff = v3i - v3Mean;
            double dDistSq = v3Diff.dot(v3Diff);
            if (dDistSq == 0.0)
                continue;
            double dNormDist = fabs(v3Diff.dot(v3Normal));

            if (dNormDist > thr)
                dNormDist = thr;
            dSumError += dNormDist;
        }
        if (dSumError < dBestDistSquared) {
            dBestDistSquared = dSumError;
            v3BestMean = v3Mean;
            v3BestNormal = v3Normal;
        }
    }

    // Done the ransacs, now collect the supposed inlier set
    for(unsigned int i=0; i<nPoints; i++) {
        Eigen::Vector3f v3i = cloud_in->points[i].getVector3fMap();
        Eigen::Vector3f v3Diff = v3i - v3BestMean;
        double dDistSq = v3Diff.dot(v3Diff);
        if (dDistSq == 0.0)
            continue;
        double dNormDist = fabs(v3Diff.dot(v3BestNormal));
        if (dNormDist < thr) {
            pcl::PointXYZ pt3;
            pt3.getVector3fMap() = v3i;
            cloud_out->points.push_back(pt3);
        }
    }

    return cloud_out;
}

void calc_plane_normal(const PointCloudXYZ::Ptr cloud_in, pcl::Normal &normal) {
    // With these inliers, calculate mean and cov
    Eigen::Vector3f v3_mean = Eigen::Vector3f::Zero();
    for(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::iterator it = cloud_in->begin();
        it != cloud_in->end(); it++) {
        v3_mean += it->getVector3fMap();
    }
    v3_mean *= (1.0 / cloud_in->size());

    Eigen::Matrix3f m3_cov = Eigen::Matrix3f::Zero();
    for(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::iterator it = cloud_in->begin();
        it != cloud_in->end(); it++) {
        Eigen::Vector3f v3_diff = it->getVector3fMap() - v3_mean;
        m3_cov += v3_diff * v3_diff.transpose();
    }

    // Find the principal component with the minimal variance: this is the plane normal
    Eigen::EigenSolver<Eigen::Matrix3f> es(m3_cov);
    Eigen::Matrix3f D = es.pseudoEigenvalueMatrix();
    Eigen::Matrix3f V = es.pseudoEigenvectors();

    std::cout << "D: \n" << D << std::endl;

    Eigen::Vector3f v3normal = V.col(2); // eigen vector with the minimal eigen value

    // Use the version of the normal which points towards the cam center
    if(v3normal[2] > 0)
        v3normal *= -1.f;

    normal = pcl::Normal(v3normal[0], v3normal[1], v3normal[2]);
}