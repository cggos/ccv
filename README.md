# CCV (Chenguang Computer Vision)

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()

CCV (Chenguang Computer Vision) is a comprehensive computer vision library that provides foundational mathematics, kinematics, dynamics, estimation, and core CV algorithms.

## Overview

The core algorithmic implementation is designed to have **no third-party dependencies** (no OpenCV, Eigen, or PCL in core logic), ensuring it is highly portable and cross-platform. Alongside the core library, this repository contains a variety of wrappers, demos, and GUI applications (C++, Python, Java, C#) demonstrating integrations with major industry tools like OpenCV, PCL, OpenGL, Pangolin, Qt, and ROS.

### Key Features

*   **Zero Dependencies:** Core modules are written from scratch for maximum portability.
*   **Modular Design:** Compile only what you need using CMake flags.
*   **High Performance:** Uses a `FLOAT` macro to toggle between single and double precision.
*   **Robust Testing:** Validated against industry standards (Eigen, OpenCV) using GTest.
*   **Well Documented:** Mathematical formulas are explicitly annotated in Doxygen comments with LaTeX.

---

## Core Modules

### 📐 Maths
Basic math methods, matrices, and vectors.
- [x] Random Number Generation
- [x] Matrix & Vector Structures
- [ ] Interpolation (Linear, Bilinear)
- [ ] SVD Decomposition

<p align="center">
  <img src="docs/maths/img/maths_map.png" style="width:60%;"/>
</p>

### 🛰️ Kinematics and Dynamics
Rotation matrices, Hamilton quaternions, and Euler angles.
- [x] Rotation Matrix
- [x] Quaternion (Hamilton)
- [x] Euler Angle (12 conventions)
- [x] Converters (with small-angle approximations)

### 📈 Estimation
State estimation for SLAM, filters, and optimization.
- [x] Filters (EKF, Particle Filter)
- [x] MAP (Gauss-Newton, Levenberg-Marquardt)
- [x] Bundle Adjustment (Sparse Hessian)
- [x] Solver Interfaces (Ceres, G2O, GTSAM)

<p align="center">
  <img src="docs/estimation/img/mat_H.png" style="width:50%"/>
</p>

### 📷 Computer Vision
2D/3D data structures and core vision algorithms.
- [x] Basic Structures (Size, Point2, Point3)
- [x] 2D Image Processing (Copy, Gaussian Filter, Pyramids)
- [x] 2D Features (FAST)
- [ ] 2D Features (ORB)
- [ ] Camera Models & Binocular Stereo Vision

---

## Building and Running

### CMake (Standalone)
```bash
# Configure the project
# Flags: -DBUILD_TEST=ON, -DBUILD_DOCS=ON, -DWITH_OPENCV=ON, -DWITH_PCL=ON, -DWITH_GL=ON
cmake -S ./ -B build -DBUILD_TEST=ON

# Build and install
cmake --build build --target install/strip --parallel $(($(nproc) / 4))

# (For CMake >= 3.15)
# cmake --install build --prefix /opt/ccv
```

### ROS (using catkin_tools)
```bash
catkin build -j$(nproc) -DWITH_ROS=ON -DWITH_PCL=ON ccv
```

---

## Apps & Integrations

*   **[CVStudio](https://github.com/cggos/CVStudio):** A comprehensive GUI application for computer vision using Qt.
*   **CamKit:** A collection of camera utilities (Android, V4L2, MIPI, RealSense, ROS).
*   **Language Wrappers:**
    *   **Python:** Available via `pip install libccv`. See `python/libccv/img_match_graph.py` for match visualization.
    *   **Java/C#:** Desktop Image Processing (DIP) demos included.
*   **Industry Demos:** Integration examples with OpenCV, PCL, OpenGL, Pangolin, and FFTW.

<p align="center">
  <img src="csharp/images/dip_csharp_ubuntu.png" style="width: 70%"/>
  <br><i>C# DIP Demo on Ubuntu</i>
</p>

---

## Development

The codebase follows the **Google C++ Code Style**. For detailed guidelines on contributions, testing, and documentation, see [coding_style.md](coding_style.md).

*   **Unit Tests:** Located in `unit_test/`, powered by GTest.
*   **Documentation:** Generate Doxygen docs with `cmake --build build --target docs`.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
