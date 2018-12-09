# Camera Calibration & Rectification

<div align="center">
  <img src="./images/cam_calib_01.jpg"> <img src="./images/cam_calib_02.jpg">
</div>

-----

[TOC]

## Camera Calibration

* [Tutorial Camera Calibration](http://boofcv.org/index.php?title=Tutorial_Camera_Calibration)
* [张氏法相机标定](https://zhuanlan.zhihu.com/p/24651968)

### OpenCV
* [Interactive camera calibration application](http://docs.opencv.org/3.2.0/d7/d21/tutorial_interactive_calibration.html)
* [Calibrate fisheye lens using OpenCV](https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0)

### [camera_calibration (ROS Wiki)](http://wiki.ros.org/camera_calibration)

Supported camera model: **pinhole camera model**, which is standard in OpenCV and ROS

* monocular camera
```bash
rosrun camera_calibration cameracalibrator.py \
    --size 11x8 \
    --square 0.03 \
    image:=/camera/image_raw
```

* stereo camera
```bash
rosrun camera_calibration cameracalibrator.py \
    --approximate=0.1 \
    --size 11x8 \
    --square 0.03 \
    right:=/stereo/right/image_raw \
    left:=/stereo/left/image_raw
```

### [Camera Calibration Toolbox for Matlab](http://www.vision.caltech.edu/bouguetj/calib_doc/)

![matlab_calib.gif](./images/matlab_calib.gif)

### [CamOdoCal](https://github.com/hengli/camodocal)
**Automatic Intrinsic and Extrinsic Calibration of a Rig with Multiple Generic Cameras and Odometry.**  

This C++ library supports the following tasks:  
* Intrinsic calibration of a generic camera.  
* Extrinsic self-calibration of a multi-camera rig for which odometry data is provided.  
* Extrinsic infrastructure-based calibration of a multi-camera rig for which a map generated from task  2 is provided.

The intrinsic calibration process computes the parameters for one of the following three camera models:  

* **Pinhole camera model**
* **Unified projection model** (C. Mei, and P. Rives, Single View Point Omnidirectional Camera Calibration from Planar Grids, ICRA 2007)
* **Equidistant fish-eye model** (J. Kannala, and S. Brandt, A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses, PAMI 2006)

By default, the **unified projection model** is used since this model approximates a wide range of cameras from normal cameras to catadioptric cameras. Note that in our **equidistant fish-eye model**, we use 8 parameters: k2, k3, k4, k5, mu, mv, u0, v0. k1 is set to 1.

### Kalibr

[Kalibr](https://github.com/ethz-asl/kalibr) is a toolbox that solves the following calibration problems:  

* Multiple camera calibration
* Camera-IMU calibration
* Rolling Shutter Camera calibration

### GML C++ Camera Calibration Toolbox
[GML Camera Calibration toolbox](http://graphics.cs.msu.ru/en/node/909) is a free functionally completed tool for cameras' calibrating. You can easy calculate intrinsic and extrinsic camera parameters after calibrating.

## Image Rectification

* [image_proc (ROS wiki)](http://wiki.ros.org/image_proc)
  - Single image rectification and color processing.

* [ethz-asl/image_undistort](https://github.com/ethz-asl/image_undistort)
  - A compact package for undistorting images directly from **kalibr calibration files**, Can also perform dense stereo estimation

* [Stereo Fisheye Rectification](https://github.com/ShreyasSkandanS/stereo_fisheye_rectify)
