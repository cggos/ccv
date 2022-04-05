#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <ctime>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    srand(time(NULL));  // seeds rand() with the system time

    time_t begin, end;
    begin = clock();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 400000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    // K nearest neighbor search
    int K = 5;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
                      << " " << cloud->points[pointIdxNKNSearch[i]].y
                      << " " << cloud->points[pointIdxNKNSearch[i]].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    end = clock();
    double Times = double(end - begin) / CLOCKS_PER_SEC;  //将clock()函数的结果转化为以秒为单位的量

    std::cout << "time: " << Times << "s" << std::endl;

    return 0;
}