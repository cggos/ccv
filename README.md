# cgocv_app

apps based on [cgocv](https://github.com/cggos/cgocv), computer vision docs and demos based on opencv and pcl using c++, matlab or c#

-----

## Overview

* **cv_docs**: Documentations about Computer Vision
* **cv_py**: Programming Computer Vision with Python
* **dip_csharp**: DIP GUI Application with C#
* **opencv_demo**: OpenCV practice Demos
* **opengl_demos**: OpenGL practice Demos
* **pcl_visualizer_qt**: PCL Visualizer with Qt
* **ros_wrapper**
  - **camera_roswrapper**
  - **pointcloud_ros_wrapper**
  - **stereo_reconstruct**

## Build

### ros_wrapper
* download dependencies
  ```bash
  cd ros_wrapper/src
  wstool init . ../ros_install.yaml
  wstool update
  ```
