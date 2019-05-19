#  Binocular Stereoscopic Vision

* [Stereo Vision and Applications](http://vision.deis.unibo.it/~smatt/stereo.htm)
* [双目立体视觉三维重建](https://blog.csdn.net/u011178262/article/details/81156412)
* [真实场景的双目立体匹配（Stereo Matching）获取深度图详解](https://www.cnblogs.com/riddick/p/8486223.html)
* [OpenCV+OpenGL 双目立体视觉三维重建](https://blog.csdn.net/wangyaninglm/article/details/52142217)
* [Stereo Vision](https://sites.google.com/site/5kk73gpu2010/assignments/stereo-vision#TOC-Update-Disparity-Map)
* [Stereo Vision Tutorial - Part I](http://mccormickml.com/2014/01/10/stereo-vision-tutorial-part-i/)
* [Depth from Stereo (librealsense)](https://github.com/IntelRealSense/librealsense/blob/master/doc/depth-from-stereo.md)
* [双目slam基础](https://github.com/Ewenwan/MVision/blob/master/vSLAM/%E5%8F%8C%E7%9B%AEslam%E5%9F%BA%E7%A1%80.md)

-----

[TOC]

## Stereo Rectify

* [Epipolar Rectification](http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO2/rectif_cvol.html)
* Bouguet极线校正

理想双目系统：两摄像机图像平面平行，光轴和图像平面垂直，极点处于无线远处，此时点(x0,y0)对应的级线就是y=y0

## Stereo Match

* [立体匹配（知乎）](https://www.zhihu.com/topic/20083757/hot)
* [双目匹配与视差计算](https://blog.csdn.net/pinbodexiaozhu/article/details/45585361)

### BM

* stereoBM

### SGM

* [一文读懂经典双目稠密匹配算法SGM](https://zhuanlan.zhihu.com/p/49272032)
* [fixstars/libSGM](https://github.com/fixstars/libSGM): Stereo Semi Global Matching by cuda

### Census

* Cencus方法保留了窗口中像素的位置特征,并且对亮度偏差较为鲁棒,简单讲就是能够减少光照差异引起的误匹配

* Census原理：在视图中选取任一点，以该点为中心划出一个例如3 × 3 的矩形，矩形中除中心点之外的每一点都与中心点进行比较，灰度值小于中心点记为1，灰度大于中心点的则记为0，以所得长度为 8 的只有 0 和 1 的序列作为该中心点的 census 序列,即中心像素的灰度值被census 序列替换。

### AD-Census
* [StereoVision-ADCensus](https://github.com/DLuensch/StereoVision-ADCensus)
* [ADCensus Stereo Matching 笔记](https://www.cnblogs.com/sinbad360/p/7842009.html)

### CNN

* [Pyramid Stereo Matching Network](https://github.com/JiaRenChang/PSMNet)
