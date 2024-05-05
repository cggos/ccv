# CCV

Chenguang Computer Vision

---

## Overview

* **core**: core computer vision library with C++ or OpenCV
* **libs**: cv demos with OpenCV, FFTW, PCL, OpenGL, Pangolin
* **apps**: cv demos with the core library or with opencv using C++, Qt, C#, Java

## Build

```bash
# for CMake Plain Project (No ROS)
mkdir build 
cd build
cmake .. [-DBUILD_TEST=ON | -DBUILD_DOCS=ON]
make -j$(nproc)

# for ROS Project, use catkin_tools
catkin build -j$(nproc) -DWITH_ROS=ON [-DWITH_PCL=ON] <package-name>
```

## Core

* Maths

* Kinematics

* Computer Vision

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

## Scripts

### Python

* View Image Matches with **GraphViz**
  ```sh
  scripts/cv_py/img_match.py
  ```

  <p align="center">
    <img src="./python/imgs/imgmatch_graphviz.png" style="width: 80%"/>
  </p>

### Matlab
