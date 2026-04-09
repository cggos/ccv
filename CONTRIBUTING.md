# Contributing Guide

感谢您对 CCV (Chenguang Computer Vision) 项目的关注！本指南将帮助您了解如何参与项目开发。

---

## 快速开始

1. **阅读 [AGENTS.md](AGENTS.md)** - AI 代理会话协议
2. **阅读 [CLAUDE.md](CLAUDE.md)** - 详细开发指南
3. **运行 `./scripts/init.sh`** - 初始化开发环境
4. **运行 `./scripts/check.sh`** - 提交前完整检查

---

## 工程架构原则

### 核心设计理念

| 原则 | 说明 |
|------|------|
| **零第三方依赖** | 核心算法代码不依赖 OpenCV、Eigen、PCL 等库 |
| **跨平台** | 使用标准 C++17，支持 Linux、macOS |
| **模块化** | CMake 开关控制，每个模块输出独立 so 文件 |
| **精度可配置** | `FLOAT` 宏控制单精度/双精度，默认 `float` |
| **安全比较** | 使用 `eps` 常量处理接近零的浮点数比较 |

### 命名空间规范

- **所有代码必须在 `cg` 命名空间内**
- 子模块使用嵌套命名空间，如 `cg::maths`

---

## 代码风格规范

### C++ 代码风格

- **Google C++ Style Guide** (已通过 `.clang-format` 配置)
- **C++17 标准** (最低要求)
- **120 字符行长度限制**
- **4 空格缩进**（非 Tab）

### 格式化命令

```bash
# 自动格式化所有代码
./scripts/format.sh

# 手动格式化 C++
find . -name "*.cpp" -o -name "*.cc" -o -name "*.h" | xargs clang-format -i

# 检查格式（CI 使用）
clang-format --dry-run --Werror <file>
```

---

## 模块开发规范

### 模块分类

#### 1. 基础数学 (maths)
- **要求**: 不依赖 Eigen
- **内容**: 向量、矩阵、SVD、随机数、插值
- **输出**: `libccv_maths.so`

#### 2. 运动学与动力学 (kd)
- **要求**: 统一右手坐标系
- **内容**:
  - 旋转矩阵
  - Hamilton 四元数（统一规范）
  - 欧拉角（12 种约定）
  - 转换器（包含小角度近似）
- **输出**: `libccv_kd.so`

#### 3. 计算机视觉 (cv)
- **要求**: 不依赖 OpenCV、PCL
- **内容**:
  - 基础类型: Size, Point2, Point3, Rect
  - 2D 图像处理: Image 类（对标 cv::Mat）
  - 滤波: 均值、高斯
  - 特征: FAST、ORB
  - 相机模型、双目视觉
- **输出**: `libccv.so`

#### 4. 估计 (est)
- **内容**: EKF、粒子滤波、BA、优化器接口

### 模块依赖关系

```
ccv (cv) → ccv_kd (kd) → ccv_maths (maths)
```

### 新增模块步骤

1. **头文件**: `include/ccv/<module>/`
2. **实现**: `src/<module>/`
3. **CMake**: 添加到 `CMakeLists.txt`，输出独立 so
4. **测试**: 在 `unit_test/` 添加 GTest 测试
5. **文档**: 更新 `docs/` 和 `mkdocs.yml`

---

## 数据类型规范

### 浮点数类型

```cpp
// 使用 FLOAT 宏定义，可切换单/双精度
typedef float FLOAT;  // 默认
// typedef double FLOAT;  // 可选

// 接近零比较
const FLOAT eps = 1e-6;
if (std::abs(val) < eps) {
    // 特殊处理
}
```

### 基础数据结构

- `cg::Size` - 宽/高
- `cg::Point2i/f/d` - 2D 点（模板类）
- `cg::Point3i/f/d` - 3D 点（模板类）
- `cg::Matrix` - 自定义矩阵（无 Eigen 依赖）
- `hpc::TScalarF` - 标量类型别名

---

## 单元测试规范

### 测试框架

- **GTest** (Google Test)
- 测试文件: `unit_test/test_*.cpp`

### 测试数据规范

| 测试类型 | 数据要求 |
|---------|---------|
| 矩阵运算 | 使用随机矩阵 |
| 图像处理 | 使用 Lena 图像或标准数据集 |
| 对比测试 | 与 Eigen、OpenCV 结果对比 |

### 测试命令

```bash
# 运行所有测试
./scripts/test.sh

# 运行单个测试套件
./unit_test/test_maths
./unit_test/test_cv
./unit_test/test_kinematics
```

### 测试编写示例

```cpp
#include <gtest/gtest.h>
#include "ccv/maths/matrix.h"

TEST(MatrixTest, Multiplication) {
    cg::Matrix A(2, 3), B(3, 2);
    // 初始化...
    cg::Matrix C = A * B;
    EXPECT_EQ(C.rows(), 2);
    EXPECT_EQ(C.cols(), 2);
    // 数值验证...
}
```

---

## 文档注释规范

### Doxygen 格式

```cpp
/**
 * @brief 简短描述
 * @param param1 参数说明
 * @return 返回值说明
 * 
 * 详细描述...
 * 
 * 数学公式（LaTeX）:
 * \f[
 *     y = Ax + b
 * \f]
 * 
 * @see 参考链接
 */
```

### 数学公式要求

1. **所有算法必须标注 LaTeX 公式**
2. **公式与代码严格对应**
3. **注明公式出处**（论文链接或书籍章节）

### 示例

```cpp
/**
 * @brief 计算旋转矩阵的逆
 * 
 * 旋转矩阵的逆等于其转置:
 * \f[ R^{-1} = R^T \f]
 * 
 * 参考: https://en.wikipedia.org/wiki/Rotation_matrix
 */
Matrix RotationMatrix::inverse() const;
```

---

## 输入输出参数规范

### 参数校验

```cpp
void processImage(const Image& input, Image& output) {
    // 输入校验
    if (input.empty()) {
        throw std::invalid_argument("Input image is empty");
    }
    
    // 尺寸校验
    if (input.size() != output.size()) {
        output.resize(input.size());
    }
    
    // 处理...
}
```

### 异常处理

- 使用 C++ 标准异常 (`std::invalid_argument`, `std::runtime_error`)
- 避免使用裸指针，优先使用智能指针
- 资源管理使用 RAII 模式

---

## 提交规范

### 提交前检查清单

```bash
./scripts/check.sh  # 必须全部通过
```

检查项:
- [ ] 代码格式符合规范 (clang-format)
- [ ] 项目可以成功构建
- [ ] 所有测试通过
- [ ] 无调试打印语句残留
- [ ] 文档已更新（如需要）

### 提交信息格式

本项目遵循 [Conventional Commits](https://www.conventionalcommits.org/zh-hans/v1.0.0/) 规范。

```
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

#### 类型 (type)

**必须**是以下之一：

| 类型 | 说明 | 示例 |
|------|------|------|
| `feat` | 新功能 | `feat: add matrix inversion` |
| `fix` | 修复 bug | `fix: correct quaternion normalization` |
| `docs` | 仅文档更改 | `docs: update API documentation` |
| `style` | 不影响代码含义的格式更改 | `style: format with clang-format` |
| `refactor` | 代码重构 | `refactor: simplify matrix multiplication` |
| `perf` | 性能优化 | `perf: optimize SVD algorithm` |
| `test` | 添加或修正测试 | `test: add unit tests for rotation matrix` |
| `chore` | 构建过程或辅助工具的变动 | `chore: update CMakeLists.txt` |
| `ci` | CI 配置更改 | `ci: add GitHub Actions workflow` |
| `build` | 影响构建系统或外部依赖 | `build: upgrade to C++17` |
| `revert` | 回滚提交 | `revert: feat: add old feature` |

#### 范围 (scope)

可选，表示影响的模块：
- `maths` - 数学模块
- `cv` - 计算机视觉模块
- `kd` - 运动学与动力学模块
- `est` - 估计模块
- `scripts` - 脚本
- `docs` - 文档

#### 描述 (description)

- 使用祈使句、现在时（"change" 而非 "changed" 或 "changes"）
- 首字母小写
- 不以句号结尾

#### 正文 (body)

- 使用祈使句、现在时
- 说明修改的动机和与之前行为的对比

#### 页脚 (footer)

- **Breaking Changes**: 以 `BREAKING CHANGE:` 开头，描述不兼容变更
- **Issues**: 引用相关 Issue，如 `Fixes #123`, `Refs #456`
- **Tasks**: 引用任务 ID，如 `Closes TASK-001`

#### 完整示例

```
feat(maths): add SVD decomposition with full U/V matrices

Implement SVD using Jacobi method for m×n matrices.
Supports both single and double precision via FLOAT macro.

Validated against Eigen::JacobiSVD with random test matrices.
Performance: ~2x slower than Eigen but zero external dependency.

Refs: #TASK-001
Closes #42
```

#### Breaking Change 示例

```
feat(cv)!: change Image class API

BREAKING CHANGE: Image constructor now requires explicit size parameter.
Old: Image img; img.load("file.jpg");
New: Image img(Size(640, 480)); img.load("file.jpg");

Migration guide: update all Image instantiations to provide size.
```

---

## Python 绑定开发

### 环境要求

- **Python 3.10+**
- 包管理: `python/pyproject.toml`

### 开发流程

```bash
cd python
pip install -e .  # 可编辑安装

# 运行测试
python -m pytest
```

### 代码风格

- **Ruff** 用于格式化和检查
- 配置见 `.pre-commit-config.yaml`

---

## 使用方式

### 作为共享库

```bash
cmake --build build --target install/strip
# 安装到 /opt/ccv
```

### 作为 Git 子模块

```bash
git submodule add https://github.com/cggos/ccv.git third_party/ccv
```

然后在主项目的 CMakeLists.txt:
```cmake
add_subdirectory(third_party/ccv)
target_link_libraries(your_target ccv)
```

---

## 获取帮助

- **Issues**: [GitHub Issues](https://github.com/cggos/ccv/issues)
- **文档**: https://cv.cgabc.xyz/
- **邮件**: cggos@outlook.com

---

**感谢您对 CCV 的贡献！**
