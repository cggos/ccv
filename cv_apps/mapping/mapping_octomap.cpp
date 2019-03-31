#include <octomap/octomap.h>

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
    
    cout<<"正在将图像转换为 Octomap ..."<<endl;
    
    octomap::OcTree tree( 0.05 );
    
    for ( int i=0; i<5; i++ )
    {
        cout<<"转换图像中: "<<i+1<<endl; 
        cv::Mat color = colorImgs[i]; 
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        
        octomap::Pointcloud cloud;
        
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 )
                    continue;
                if ( d >= 7000 )
                    continue;

                Eigen::Vector3d point;
                point[2] = double(d) / tum_data_rgbd.depth_scale_;
                point[0] = (u-tum_data_rgbd.cam_k_.cx)*point[2]/tum_data_rgbd.cam_k_.fx;
                point[1] = (v-tum_data_rgbd.cam_k_.cx)*point[2]/tum_data_rgbd.cam_k_.fy;

                Eigen::Vector3d pointWorld = T*point;

                cloud.push_back( pointWorld[0], pointWorld[1], pointWorld[2] );
            }
            
        // 将点云存入八叉树地图，给定原点，这样可以计算投射线
        tree.insertPointCloud( cloud, octomap::point3d( T(0,3), T(1,3), T(2,3) ) );     
    }
    
    tree.updateInnerOccupancy();

    cout<<"saving octomap ... "<<endl;
    tree.writeBinary( "octomap.bt" );

    return 0;
}
