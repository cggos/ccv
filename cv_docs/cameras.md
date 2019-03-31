# Cameras

* [Sensors/Cameras (ROS Wiki)](http://wiki.ros.org/Sensors/Cameras)

-----

[TOC]

## 1. Camera Lenses

* [Lensation](https://www.lensation.de/) provides free of charge consulting about lenses, illumination and optical components
* [dxomark](https://www.dxomark.com/): source for image quality benchmarks for phones and cameras

## Lens Knowledge

* [Norman Koren photography: images and tutorials](http://www.normankoren.com/)
* [摄像机镜头详细知识](https://zhuanlan.zhihu.com/p/29098395)
* [camera资料链接整理](https://blog.csdn.net/ccwwff/article/details/86679455)
* [摄像头模组基础扫盲](https://www.cnblogs.com/raymon-tec/p/5048632.html)
* [相机的那些事儿 - 概念、模型及标定](https://yq.aliyun.com/articles/62472)
* [如何理解 ISO、快门、光圈、曝光这几个概念？](https://www.zhihu.com/question/21427664)
* [MT9V034 Tutorial：如何使用全局快门摄像头](https://zhuanlan.zhihu.com/p/34516668)
* [Introduction to Shutter Speed in Photography](https://photographylife.com/what-is-shutter-speed-in-photography)
* [HUTTER SPEED: A BEGINNER'S GUIDE](https://www.photographymad.com/pages/view/shutter-speed-a-beginners-guide)
* [EXPOSURE, APERTURE AND SHUTTER SPEED EXPLAINED](https://www.photographymad.com/pages/view/exposure-aperture-shutter-speed)
* [摄影知识普及：如何用好滤光镜，想进一步玩好摄影必看！](http://www.sohu.com/a/168545276_374721)

## Lens Types

* Fisheye Lens: Its not unusual for a fisheye lens to have a FOV of 185 degrees.

## Lens Test

* [ISO 12233 Test Chart](http://www.graphics.cornell.edu/~westin/misc/res-chart.html)
* [Setting Up an Optical Testing Station](https://www.lensrentals.com/blog/2014/02/setting-up-an-optical-testing-station/)


## 2. Industrial Camera

* [iDS](https://en.ids-imaging.com/home.html)
* [1stVision](https://www.1stvision.com/)
* [MatrixVision](https://www.matrix-vision.com)


## 3. Digital Camera
* [gPhoto](http://www.gphoto.org/) is a free, redistributable, ready to use set of digital camera software applications for Unix-like systems
* [digiCamControl](http://digicamcontrol.com/): An innovative and easy to use solution for complex camera control!
* [DSLR Controller](http://www.dslrcontroller.com/) was the first and is still the best app to fully control your Canon EOS DSLR from your Android device, with nothing more than a USB cable.
* [Generic PTP control of digital cameras](https://www.circuitsathome.com/camera-control/generic-ptp-control-of-digital-cameras/)  
![digital_cam_ctrl_p100.jpg](./images/digital_cam_ctrl_p100.jpg)


## 4. Camera Modules

* [In Search of a Better Serial Camera Module](http://sigalrm.blogspot.com/2013/07/in-search-of-better-serial-camera-module.html)

### [CMUcam](http://www.cmucam.org/)
**Open Source Programmable Embedded Color Vision Sensors**, The first CMUcam made its splash in 1999 as a CREATE Lab project.

* [The CMUcam1 Vision Sensor](https://www.cs.cmu.edu/~cmucam/qanda.html)  
![CMUcam1_B.JPG](./images/CMUcam1_B.JPG)

#### [Pixy](https://pixycam.com/)

![pixy_cam.jpg](./images/pixy_cam.jpg)

**Pixy** is **the fifth version of the CMUcam, or CMUcam5**, but “Pixy” is easier to say than CMUcam5, so the name more or less stuck.  Pixy got its start in 2013 as part of a successful Kickstarter campaign, and as a partnership between **Charmed Labs** and **CMU**.

**Pixy2** was announced recently as Pixy’s smaller, faster, and smarter younger sibling.  
![pixy2_cam.jpg](./images/pixy2_cam.jpg)

### [OpenMV](https://openmv.io/)
The OpenMV(**Open-Source Machine Vision**) project aims at making machine vision more accessible to beginners by developing a user-friendly, open-source, low-cost **machine vision platform**.  

OpenMV cameras are programmable in **Python3** and come with an extensive set of **image processing functions** such as face detection, keypoints descriptors, color tracking, QR and Bar codes decoding, AprilTags, GIF and MJPEG recording and more.  

![openmv_cam.jpg](./images/openmv_cam.jpg)

### [NXTCam-v4](http://www.mindsensors.com/ev3-and-nxt/14-vision-subsystem-camera-for-nxt-or-ev3-nxtcam-v4)
Vision Subsystem - Camera for NXT or EV3 (NXTCam-v4)  

![nxtcam_v4.jpg](./images/nxtcam_v4.jpg)

### [JeVois Smart Machine Vision Camera](http://jevois.org/)

Open-source quad-core camera effortlessly adds powerful machine vision to all your PC, Arduino, and Raspberry Pi projects.

![jevois.png](./images/jevois.png)


## 5. 3D (Depth) Cameras

### Structure Light Camera

#### Kinect
* [Kinect for windows微软中国体感官方网站](http://www.k4w.cn/)
* [OpenKinect](https://openkinect.org/wiki/Main_Page) is an open community of people interested in making use of the amazing Xbox Kinect hardware with our PCs and other devices.
* [Kinect V1 and Kinect V2 fields of view compared](http://smeenk.com/kinect-field-of-view-comparison/)
* [Ubuntu + Kinect + OpenNI + PrimeSense](http://mitchtech.net/ubuntu-kinect-openni-primesense/)
* [【翻译】Kinect v1和Kinect v2的彻底比较](http://www.cnblogs.com/TracePlus/p/4136297.html)
* [code-iai/iai_kinect2](https://github.com/code-iai/iai_kinect2): Tools for using the Kinect One (Kinect v2) in ROS

#### Orbbec Astra Camera
* [astra_camera (ROS Wiki)](http://wiki.ros.org/astra_camera)
* [ROS wrapper for Astra camera](https://github.com/orbbec/ros_astra_camera)

#### ASUS Xtion 2
* https://www.asus.com/3D-Sensor/

### Stereo Camera

#### Realsense Camera
* [realsense_camera (ROS Wiki)](http://wiki.ros.org/realsense_camera)
* [Intel® RealSense­™ Camera ZR300](https://software.intel.com/en-us/realsense/zr300)
* [ethz-asl/maplab_realsense](https://github.com/ethz-asl/maplab_realsense): Simple ROS wrapper for the Intel RealSense driver with a focus on the ZR300

#### ZED Stereo Camera
* [StereoLabs](https://www.stereolabs.com/)

#### FLIR Bumblebee
* [立体视觉](https://www.ptgrey.com/stereo-vision-cameras-systems)

#### Tara
* [Tara - USB 3.0 Stereo Vision Camera](https://www.e-consystems.com/3D-USB-stereo-camera.asp)

### Event Camera
* [Event Camera动态视觉传感器，让无人机用相机低成本进行导航](https://www.leiphone.com/news/201709/LkfPqS60ZYgmXk8x.html)
