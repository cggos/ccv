# DIP Demo with Qt and OpenCV

Digital Image Processing Demonstration with Qt.

---

<p align="center">
  <img src="imgs/dip_demo.jpg"/>
</p>

## Requirements

tested on Ubuntu 16.04 and Ubuntu 18.04

* Qt5
* OpenCV 2 (or above)

## Build & Run

* GUI
  - Qt Creator

* CLI
  
  - qmake
    ```sh
    mkdir build
    cd build

    qmake ..
    make -j4

    # run    
    ../Output/DIPDemoQt
    ```
  
  - cmake
    ```sh
    mkdir build
    cd build

    cmake ..
    make -j4

    # run    
    ./DIPDemoQt
    ```

## Summary of *Computer Vision with OpenCV 2*

Source Code of the Book: [https://github.com/vinjn/opencv-2-cookbook-src](https://github.com/vinjn/opencv-2-cookbook-src "OpenCV 2 计算机视觉编程手册")

### Key Points

* Salt and pepper noise
* Color reduce
* Image sharpening
* Color model transformation
* Histogram
* Binary image
* Look up table
* Histogram equalization
* Back Project
* MeanShift
* CamShift
* Gaussian Blur(Gaussian Distribution,Gaussian Function)
* Median filtering
* Mean filtering

## Skin Detection

* [Skin Color Thresholding with OpenCV](http://www.bytefish.de/blog/opencv/skin_color_thresholding/)
