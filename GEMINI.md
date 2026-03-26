# CCV (Chenguang Computer Vision)

## Project Overview

CCV (Chenguang Computer Vision) is a comprehensive computer vision library that provides foundational mathematics, kinematics, dynamics, estimation, and core CV algorithms. The core algorithmic implementation is designed to have **no third-party dependencies**, ensuring it is highly portable and cross-platform. 

Alongside the core library, the repository contains a variety of wrappers, demos, and GUI applications (written in C++, Python, Java, and C#) demonstrating integrations with major industry tools such as OpenCV, PCL, OpenGL, Pangolin, Qt, and ROS (Robot Operating System). The project's architecture is modular, allowing users to compile specific components based on their needs via CMake flags.

### Core Modules
* **Maths:** Basic math methods, matrices, vectors, SVD, and interpolation.
* **Kinematics & Dynamics:** Rotation matrices, quaternions (Hamilton), Euler angles, and their converters (with approximate treatment for small angles).
* **Estimation:** State estimation for SLAM, including Filters (EKF, Particle), MAP (GN, LM), Bundle Adjustment, and interfaces to solvers (Ceres, G2O, GTSAM).
* **Computer Vision:** 2D/3D data structures (Image, PointCloud), filtering (Mean, Gaussian), 2D features (FAST, ORB), and binocular stereo vision.

## Building and Running

The project can be built as a standalone CMake project or as a ROS package using `catkin`.

### CMake Plain Project (No ROS)
```bash
# Configure the project (Optional flags: -DBUILD_TEST=ON, -DBUILD_DOCS=ON, -DWITH_OPENCV=ON, -DWITH_PCL=ON, -DWITH_GL=ON)
cmake -S ./ -B build -DBUILD_TEST=ON -DBUILD_DOCS=ON

# Build and install
cmake --build build --target install/strip --parallel $(($(nproc) / 4))

# (For CMake >= 3.15)
# cmake --install build --prefix /opt/ccv
```

### ROS Project (using catkin_tools)
```bash
catkin build -j$(nproc) -DWITH_ROS=ON -DWITH_PCL=ON ccv
```

## Development Conventions

The codebase adheres to strict engineering and mathematical guidelines:

* **Coding Style:** Follows the **Google C++ Code Style**.
* **Zero Dependencies:** Core algorithm code must not rely on third-party libraries (no OpenCV, Eigen, or PCL in the core logic).
* **Data Types:** Uses a `FLOAT` macro to easily toggle between single (`float`) and double (`double`) precision (defaults to `float`). Uses an `eps` constant for safe floating-point comparisons near zero.
* **Testing Practices:** Comprehensive unit testing using **GTest**. Test data utilizes random matrices for math operations and standard datasets (e.g., Lena) for image processing. Results are actively benchmarked and validated against established libraries like Eigen, OpenCV, or CImg.
* **Documentation:** Code relies on **Doxygen** combined with **LaTeX**. Mathematical formulas must be explicitly annotated in the code comments and strictly map to the corresponding implementation, often citing algorithm sources.
* **Deployment:** Core modules output shared libraries designed to be easily consumed via git submodules.
