# Third-party Library Integrations & Demos

This directory contains demonstration applications and integration examples that bridge the core CCV library with popular third-party computer vision, robotics, and visualization frameworks.

---

## Libraries Included

### [OpenCV](./opencv)
A comprehensive collection of classical computer vision demos.
- **Features:** Face detection, edge detection, feature matching (ORB), camera calibration, and image transformations.
- **Enable via CMake:** `-DWITH_OPENCV=ON`

### [PCL (Point Cloud Library)](./pcl)
Demonstrations for 3D point cloud processing and visualization.
- **Includes:** Basic PCL demos, ROS-integrated point cloud processing, and Qt-based visualizers.
- **Enable via CMake:** `-DWITH_PCL=ON`

### [Pangolin](./pangolin)
Lightweight OpenGL-based visualization, commonly used for SLAM.
- **Demos:** Multi-view display, multithreading, camera pose drawing, and SLAM trajectory visualization.

### [OpenGL](./opengl)
Low-level 3D graphics demonstrations.
- **Enable via CMake:** `-DWITH_GL=ON`

### [Open3D](./open3d)
Python-based 3D data processing using Open3D.
- **Scripts:** TSDF integration, RGB-D processing, and basic 3D I/O.

### [FFTW](./fftw)
Fast Fourier Transform examples for frequency domain image processing.

---

## Build Instructions

Most library demos can be toggled via CMake flags during the main project configuration.

```bash
# Example: Enable OpenCV and PCL support
cmake -S .. -B ../build -DWITH_OPENCV=ON -DWITH_PCL=ON
cmake --build ../build
```

For specific requirements (e.g., system dependencies), please refer to the `README.md` within each library's subdirectory.
