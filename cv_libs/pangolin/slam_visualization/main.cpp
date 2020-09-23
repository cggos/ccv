#include <pangolin/pangolin.h>
#include <stdio.h>
#include <unistd.h>

#include <queue>
#include <thread>

#include "slam_visualizer.h"

using namespace std;

int main(int argc, char **argv) {
    SlamVisualizer visualizer(1504, 960);

    queue<string> imgFileNames;
    queue<ulong> imgTimeStamps;
    vector<Eigen::Vector3d> traj;

    FILE *fp_gt = fopen("/home/cg/projects/datasets/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv", "r");
    FILE *fp_img = fopen("/home/cg/projects/datasets/euroc/MH_01_easy/mav0/cam0/data.csv", "r");
    if (fp_gt == nullptr || fp_img == nullptr) {
        cout << "failed to open file !\n";
        return -1;
    }
    char fl_buf[1024];
    fgets(fl_buf, sizeof(fl_buf), fp_img);  // 跳过第一行
    while (!feof(fp_img)) {
        char filename[23];
        ulong timestamp;
        fscanf(fp_img, "%lu,%s", &timestamp, filename);
        imgTimeStamps.push(timestamp);
        imgFileNames.push(string(filename));
    }
    fgets(fl_buf, sizeof(fl_buf), fp_gt);  // 跳过第一行

    visualizer.initDraw();

    while (!feof(fp_gt)) {
        Eigen::Quaterniond quat;
        Eigen::Vector3d pos;
        cv::Mat img;
        {
            ulong time_stamp(0);
            double px(0.), py(0.), pz(0.);
            double qw(0.), qx(0.), qy(0.), qz(0.);
            double vx(0.), vy(0.), vz(0.);
            double bwx(0.), bwy(0.), bwz(0.), bax(0.), bay(0.), baz(0.);
            fscanf(fp_gt, "%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                   &time_stamp, &px, &py, &pz,
                   &qw, &qx, &qy, &qz,
                   &vx, &vy, &vz,
                   &bwx, &bwy, &bwz,
                   &bax, &bay, &baz);
            quat = Eigen::Quaterniond(qw, qx, qy, qz);
            pos = Eigen::Vector3d(px, py, pz);
            traj.push_back(pos);

            // 弹出当前时刻之前的图像
            double imu_time = (double)time_stamp / 1e9;
            double img_time = (double)imgTimeStamps.front() / 1e9;
            if (imu_time > img_time) {
                imgTimeStamps.pop();
                imgFileNames.pop();
            }
            string img_file = "/home/cg/projects/datasets/euroc/MH_01_easy/mav0/cam0/data/" + imgFileNames.front();
            img = cv::imread(img_file, CV_LOAD_IMAGE_COLOR);
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        visualizer.activeAllView();
        visualizer.registerUICallback();
        visualizer.displayData(pos, quat);

        visualizer.drawCoordinate();
        visualizer.drawCamWithPose(pos, quat);
        visualizer.drawTraj(traj);
        visualizer.displayImg(img, img);  // 由于数据集没有跟踪图像，这里两幅图像显示一样

        pangolin::FinishFrame();

        if (pangolin::ShouldQuit())
            break;

        usleep(5000); // 挂起5ms
    }

    cin.get();

    return 0;
}
