# CCV Core

----

## Build & Run

* build

  ```bash
  # for CMake Plain Project (No ROS)
  mkdir build 
  cd build
  cmake .. [-DBUILD_TEST=ON | -DBUILD_DOCS=ON]
  make -j$(nproc)
  
  # for ROS Project, use catkin_tools
  catkin build -j$(nproc) -DWITH_ROS=ON <package-name>
  ```

* run unit test

  ```sh
  cd build
  ./unit_test/test_xxx
  ```

## Modules

### Maths

* Basic Math Methods
  - [x] Random Number
  - [ ] Interpolation
    - [ ] Linear Interpolation
    - [ ] Bilinear Interpolation

* Data Structures & Methods
  - [x] Matrix
  - [x] Vector

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

### Kinematics

**Note**: approximate treatment about **small angle**.

* [x] Rotation Matrix
* [x] Quarternion (Hamilton)
* [x] Euler Angle
* [x] Convertor
