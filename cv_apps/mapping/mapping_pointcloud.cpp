#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "tum_data_rgbd.h"

using namespace std;

int main( int argc, char** argv )
{
    const int count = 10;

    cg::TUMDataRGBD tum_data_rgbd("/home/cg/dev_sdb/datasets/TUM/RGBD-SLAM-Dataset/rgbd_dataset_freiburg1_xyz/");

    vector<cv::Mat> colorImgs, depthImgs;
    vector<Eigen::Isometry3d> poses;

    for(int i=0; i<count; ++i) {
        cv::Mat img_color;
        cv::Mat img_depth;
        Eigen::Isometry3d pose;

        if (!tum_data_rgbd.get_rgb_depth_pose(img_color, img_depth, pose)) {
            std::cerr << "get_rgb_depth_pose failed!" << std::endl;
            return -1;
        }

        colorImgs.push_back(img_color);
        depthImgs.push_back(img_depth);
        poses.push_back(pose);
    }

    cout<<"正在将图像转换为点云..."<<endl;
    
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    
    PointCloud::Ptr pointCloud( new PointCloud );
    for ( int i=0; i<count; i++ )
    {
        PointCloud::Ptr current( new PointCloud );

        cout<<"转换图像中: "<<i+1<<endl;

        cv::Mat color = colorImgs[i]; 
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];

        for ( int v=0; v<color.rows; v++ )
        {
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u];
                if ( d==0 )
                    continue;
                if ( d >= 7000 )
                    continue;

                Eigen::Vector3d point; 
                point[2] = double(d) / tum_data_rgbd.depth_scale_;
                point[0] = (u-tum_data_rgbd.cam_k_.cx)*point[2]/tum_data_rgbd.cam_k_.fx;
                point[1] = (v-tum_data_rgbd.cam_k_.cx)*point[2]/tum_data_rgbd.cam_k_.fy;

                Eigen::Vector3d pointWorld = T*point;
                
                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];

                current->points.push_back( p );
            }
        }

        PointCloud::Ptr tmp ( new PointCloud );
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter( *tmp );

        (*pointCloud) += *tmp;
    }
    
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;

    PointCloud::Ptr tmp ( new PointCloud );
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setLeafSize( 0.01, 0.01, 0.01 );
    voxel_filter.setInputCloud( pointCloud );
    voxel_filter.filter( *tmp );

    tmp->swap( *pointCloud );
    
    cout<<"滤波之后，点云共有"<<pointCloud->size()<<"个点."<<endl;
    
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );

    return 0;
}
