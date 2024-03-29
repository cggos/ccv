# Coding Style

---

## 工程特点（要求）

* cmake工程
* 算法核心代码无第三方依赖，可跨平台使用
* 详细的单元测试，矩阵测试数据使用随机矩阵，图片测试数据使用经典的lena或标准数据集
* 输入输出参数判断
* 浮点型数据类型使用 宏FLOAT，可以单精度float和双精度double切换，默认float
* 定义eps，对接近零的浮点型数据做判断，并进行特殊处理或计算（如果需要）
* 最外层命名空间

## 主要内容（范围）

### 模块分类

* 基本数据结构
  - common.h
* 基本数学运算 (No Eigen)
  - 向量
  - matrix (用Viso2中的)
  - 反对称矩阵
  - 插值算法interpolation algorithm: 线性linear, 双线性bilinearity
* 计算机视觉computer vision (No OpenCV & PCL)
  - 基本数据结构
    - Size
    - RGB
    - Rect
  - 2D image process (对标实现 cv::Mat)  
    - 实现2D Image类
    - 拷贝操作
    - 颜色空间转换: YUV-->RGB, RGB-->grayscale
    - 放大缩小
    - 图像滤波: mean, gaussian
  - 3D pointcloud
  - camera model
  - stereo vision
    - stereo match
    - disparity --> depth image
* 运动学kinematics (统一右手系)
  - 旋转矩阵rotation matrix
  - 单位四元数quarternion（统一Hamilton四元数）
    - quarternion结构体
    - 归一化操作
    - 其他运算，根据需要补充
  - 转换
    - euler angles <--> rotation matrix (12 methods)
    - quarternion <--> rotation matrix
  - 注意：小角度的近似运算

### 模块说明

* 每个模块还要实现各自专用的 数据结构
* 根据使用需求，通过设置cmake的开关选项，决定编译哪个模块，每个模块输出一个so文件

### 


## 单元测试

* gtest
* 使用OpenCV或者CImg等图像库作为数据的输入输出接口，并做对比测试
* 数学算法测试使用Eigen等库做对比测试
* 矩阵测试数据使用随机矩阵，图片测试数据使用经典的lena或标准数据集

## 文档注释

* doxygen with latex
* 注明公式(LaTeX)，跟代码严格对应
* 注明公式或算法出处（链接）


## 代码规范

* google code style

## 输出&使用

* 输出： shared library 
* 使用： git submodule的方式