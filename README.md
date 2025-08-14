# CCV

Chenguang Computer Vision

---

## Overview

* **core**: core computer vision library, including maths, kinematics and dynamics, estimation, etc.
* **libs**: cv demos with OpenCV, FFTW, PCL, OpenGL, Pangolin
* **apps**: cv demos with the core library or with opencv using C++, Qt, C#, Java

## Build

```bash
# for CMake Plain Project (No ROS)
cmake -S ./ -B build [-DBUILD_TEST=ON | -DBUILD_DOCS=ON]
cmake --build build --target install/strip --parallel $(($(nproc) / 4))
# after cmake 3.15
# cmake --install build --prefix ${INSTALL_DIR}

# for ROS Project, use catkin_tools
catkin build -j$(nproc) -DWITH_ROS=ON [-DWITH_PCL=ON] <package-name>
```

## Core Modules

### Maths

* Basic Math Methods
  - [x] Random Number
  - [ ] Interpolation
    - [ ] Linear Interpolation
    - [ ] Bilinear Interpolation

* Data Structures & Methods
  - [x] Matrix
  - [x] Vector

<p align="center">
  <img src="docs/maths/img/maths_map.png" style="width:80%;"/>
</p>

- https://github.com/cggos/suitesparse_android

### Kinematics and Dynamics

**Note**: approximate treatment about **small angle**.

* [x] Rotation Matrix
* [x] Quarternion (Hamilton)
* [x] Euler Angle
* [x] Convertor

### Estimation

State Estimation for SLAM: Filter(EKF, Particle Filter), MAP(GN, LM), Solver(Ceres-Solver, G2O, GTSAM), Bundle Adjustment

#### Bundle Adjustment

Sparse Hessian matrix

<p align="center">
  <img src="docs/estimation/img/mat_H.png" style="width:60%"/>
</p>

### Computer Vision

* Data Structure & Methods
  - [x] Size
  - [x] Point2
  - [ ] RGB
* 2D Image & Methods
  - [ ] YImg class
    - [x] Copy
    - [ ] ROI Extraction
    - [ ] Zoom In & Out
    - [ ] Mean Filter
    - [x] Gaussian Filter
    - [x] Image Pyramid
* 2D Features
  - Key Points
    - [x] FAST
    - [ ] ORB
  - Discriptors
  - Line
    - [ ] Edge
    - [ ] Straight Line
* 3D PointCloud
  - [x] Point3
  - [ ] PointCloud
* Camera
  - [ ] Camera Models
* Binocular Stereo Vision
  - [ ] Stereo Match
  - [ ] Disparity Compute
  - [ ] Disparity --> Depth
  - [ ] Depth --> PointCloud


## Libs

* OpenCV

* FFTW

* PCL

* OpenGL

* Pangolin

## Apps

* [x] Camera Kit

  * [x] Camera App on Android
  
  * [x] Realsense Camera utils
  
  * [x] Camera driver with v4l2

  * [x] Stereo camera driver with MIPI and v4l2

  * [x] Camera utils with ROS

  * [x] Camera calib & rectify utils

* [x] [CVStudio](https://github.com/cggos/CVStudio): GUI App with Qt for Computer Vision :sunny:

  <p align="center">
    <img src="https://mirror.ghproxy.com/https://raw.githubusercontent.com/cggos/CVStudio/master/imgs/dip_demo.jpg"/>
  </p>

* [x] DIP Demo with Java
  

* [x] DIP Demo with C#

  ![dip_csharp_ubuntu.png](csharp/images/dip_csharp_ubuntu.png)

* [x] Face Detection

* [ ] Medical Imaging

* [x] Stereo Matching with OpenCL

* [x] Stereo Reconstruction with ROS

## Languages

### Python

- https://pypi.org/project/libccv/

e.g. View Image Matches with **GraphViz**

```sh
python/libccv/img_match_graph.py
```

<p align="center">
  <img src="./python/imgs/imgmatch_graphviz.png" style="width: 80%"/>
</p>

### Matlab

### Java

### C#
