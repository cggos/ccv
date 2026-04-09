# Harness Engineering 实施报告

**项目**: CCV (Chenguang Computer Vision)  
**日期**: 2026-04-09  
**目标**: 基于 Harness Engineering 最佳实践完善工程结构和工具链

---

## 执行摘要

本报告记录了基于 [Harness Engineering](https://github.com/celesteanders/harness) 最佳实践对 CCV 计算机视觉库进行的工程改进。主要实现了 AI 代理友好型开发工作流、自动化任务追踪系统和持续集成管道。

---

## 1. 背景与动机

### Harness Engineering 核心原则

Harness Engineering 是 2025-2026 年新兴的 AI 辅助软件开发方法论，核心原则包括：

1. **多代理关注点分离** - 规划、生成、评估角色分离
2. **结构化状态持久化** - 使用 JSON 而非 Markdown 存储任务状态
3. **AGENTS.md 作为地图** - 简短入口文件 (~100行) 指向详细文档
4. **一次一个任务** - 防止上下文耗尽，保持可恢复性
5. **自动化反馈循环** - 类型检查、测试、安全扫描自动拒绝无效输出
6. **Git 作为安全网** - 每个任务后提交，脚本读取历史

### CCV 项目特点

- C++ 计算机视觉库，核心零依赖设计
- 支持多语言绑定 (Python, Java, C#, MATLAB)
- 使用 CMake 构建，Google C++ 代码风格
- 已有 CLAUDE.md 但缺乏 AI 代理工作流规范

---

## 2. 实施内容

### 2.1 AI 代理入口点

**创建文件**: `AGENTS.md`

**功能**:
- 项目概览和关键原则
- 8步会话协议 (Orient → Setup → Verify → Select → Implement → Test → Update → Exit)
- 快速命令参考表
- 目录结构导航
- 紧急恢复流程

**设计决策**:
- 保持简洁 (~80行)，作为详细 CLAUDE.md 的入口
- 明确 Golden Rules: 一次一个任务、提交前测试、清晰提交信息

### 2.2 任务追踪系统

**创建目录**: `tasks/`

| 文件 | 格式 | 内容 |
|------|------|------|
| `todo.json` | JSON | 5个活跃任务 (SVD实现、ORB特征、相机模型等) |
| `done.json` | JSON | 5个已完成任务 (矩阵、FAST、四元数等) |
| `progress.md` | Markdown | 会话日志、架构决策、已知问题 |

**设计决策**:
- 使用 JSON 而非 Markdown 存储任务 - 防止模型诱导的数据损坏
- 每个任务包含 id, title, description, priority, status, module, tags
- progress.md 记录每次会话的决策和上下文

### 2.3 开发自动化脚本

**创建目录**: `scripts/`

| 脚本 | 功能 | 选项 |
|------|------|------|
| `status.sh` | 显示项目状态 | Git状态、构建状态、任务统计 |
| `init.sh` | 初始化环境 | 检查 CMake、工具、Python环境 |
| `build.sh` | 构建项目 | `--debug`, `--no-test`, `--docs`, `--with-pcl`, `--clean` |
| `test.sh` | 运行测试 | CTest + 单独测试套件 + Python测试 |
| `check.sh` | 提交前检查 | 格式检查 → 构建 → 测试 |
| `format.sh` | 格式化代码 | C++ (clang-format) + Python (ruff) |
| `clean.sh` | 清理构建产物 | 移除 build/, Python构建产物 |

**关键特性**:
- 彩色输出 (绿/黄/红) 表示状态
- 失败时立即退出 (`set -e`)
- 检查工具可用性，缺失时优雅降级

### 2.4 持续集成管道

**创建工作流**: `.github/workflows/`

| 工作流 | 触发条件 | 矩阵配置 |
|--------|----------|----------|
| `ci.yml` | push/PR to main/develop | Linux (Release/Debug × OpenCV ON/OFF) + macOS |
| `docs.yml` | push to main (docs/**) | 自动部署 MkDocs 到 GitHub Pages |

**CI 特性**:
- 多配置矩阵测试确保兼容性
- 格式检查作为门禁 (clang-format --Werror)
- Python 版本矩阵 (3.9-3.12)
- 路径过滤避免无关文件触发构建

### 2.5 CMake 预设配置

**创建文件**: `CMakePresets.json`

| 预设 | 配置 | 用途 |
|------|------|------|
| `release` | Release + tests + OpenCV | 默认开发配置 |
| `debug` | Debug + tests + OpenCV | 调试构建 |
| `minimal` | No external dependencies | 验证零依赖核心 |
| `full` | 所有功能 + 文档 | 完整功能构建 |

**使用方法**:
```bash
cmake --preset=release    # 配置
cmake --build --preset=release    # 构建
ctest --preset=release    # 测试
```

### 2.6 版本要求定义

**更新文件**: `CMakeLists.txt`, `python/pyproject.toml`, `.github/workflows/ci.yml`

**C++ 版本要求**:
```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```
- 要求 C++17 或更高版本
- 禁用编译器扩展，确保跨平台兼容性

**Python 版本要求**:
```toml
requires-python = ">=3.10"
```
- 要求 Python 3.10 或更高版本
- CI 矩阵更新: ['3.10', '3.11', '3.12', '3.13']

**init.sh 版本检查**:
- 检查 Python 版本 (3.10+)，不满足则报错退出
- 显示 C++ 编译器版本信息

### 2.7 文档重构

**重命名文件**: `coding_style.md` → `CONTRIBUTING.md`

**原因**:
- `CONTRIBUTING.md` 是开源项目标准命名
- 内容扩展包含：工程原则、代码风格、模块规范、测试要求、提交规范（遵循 Conventional Commits）
- 更全面地指导贡献者

**更新文件**: `CLAUDE.md`, `AGENTS.md`, `README.md`

**CLAUDE.md 新增内容**:
- Quick Start 部分引用 scripts/
- CMake Presets 使用说明
- 脚本选项文档
- 任务管理说明
- Harness Engineering 最佳实践章节
- **版本要求文档**: C++17+ 和 Python 3.10+ 的明确要求
- 引用 CONTRIBUTING.md 作为编码标准详细说明

---

## 3. 文件清单

### 新建文件
```
AGENTS.md
CMakePresets.json
tasks/
  ├── todo.json
  ├── done.json
  └── progress.md
scripts/
  ├── status.sh
  ├── init.sh
  ├── build.sh
  ├── test.sh
  ├── check.sh
  ├── format.sh
  └── clean.sh
.github/workflows/
  ├── ci.yml
  └── docs.yml
```

### 更新文件
```
CMakeLists.txt           (添加 C++17 版本要求)
python/pyproject.toml    (更新 Python 版本要求为 >=3.10)
.github/workflows/ci.yml (更新 Python CI 矩阵: 3.10-3.13)
CLAUDE.md                (添加 Quick Start、脚本文档、Harness Engineering 章节、版本要求、CONTRIBUTING.md 引用)
AGENTS.md                (添加版本要求说明、CONTRIBUTING.md 引用)
README.md                (更新 coding_style.md 引用为 CONTRIBUTING.md)
scripts/init.sh          (添加 Python 版本检查)
```

---

## 4. 使用指南

### 新会话开始

```bash
# 1. 查看当前状态
./scripts/status.sh

# 2. 初始化环境
./scripts/init.sh

# 3. 构建项目
./scripts/build.sh

# 4. 查看可用任务
cat tasks/todo.json | python3 -m json.tool
```

### 开发工作流

```bash
# 实现功能后
./scripts/test.sh      # 运行测试
./scripts/check.sh     # 提交前完整检查

# 如果检查失败
./scripts/format.sh    # 自动修复格式问题
# 修复其他问题后重新运行 check.sh
```

### 清理与恢复

```bash
# 构建损坏时
./scripts/clean.sh     # 清理所有构建产物
./scripts/init.sh      # 重新初始化
./scripts/test.sh      # 验证基线
```

---

## 5. 设计决策记录

### 决策 1: JSON vs Markdown 任务存储

**选择**: JSON  
**原因**: 
- 结构化数据防止模型诱导的自由格式文本损坏
- 程序化访问更可靠
- 版本控制差异更清晰

### 决策 2: 脚本与 Makefile

**选择**: Bash 脚本  
**原因**:
- 更灵活的错误处理和彩色输出
- 清晰的命令行参数解析
- 跨平台兼容性 (macOS/Linux)

### 决策 3: AGENTS.md 长度限制

**选择**: ~80行  
**原因**:
- Harness 推荐 ~100行作为快速参考
- 过长会导致关键信息淹没
- 详细内容保留在 CLAUDE.md

### 决策 4: CMake Presets 配置

**选择**: 4个预设 (release, debug, minimal, full)  
**原因**:
- 覆盖主要使用场景
- minimal 预设验证零依赖承诺
- full 预设用于 CI 和发布

---

## 6. 已知限制与未来改进

### 当前限制

1. **脚本依赖**: status.sh 需要 Python3 解析 JSON
2. **CI 环境**: 未配置 Windows 构建 (需要 Windows 运行器)
3. **Docker**: 缺少容器化开发环境

### 建议改进

1. **预提交钩子**: 将 check.sh 集成到 git pre-commit (已部分集成 conventional commits)
2. **代码覆盖率**: 添加 gcov/lcov 生成覆盖率报告
3. **静态分析**: 集成 clang-tidy, cppcheck 到 CI
4. **性能基准**: 添加性能回归测试
5. **IDE 支持**: 添加 .vscode/settings.json 模板
6. **Commitizen**: 考虑添加 commitizen 工具辅助生成规范提交信息

---

## 7. 参考资源

### Harness Engineering
- [GitHub: celesteanders/harness](https://github.com/celesteanders/harness)
- [Gist: Harness Engineering Best Practices](https://gist.github.com/celesteanders/21edad2367c8ede2ff092bd87e56a26f)
- [nxcode.io: Harness Engineering Complete Guide](https://www.nxcode.io/resources/news/harness-engineering-complete-guide-ai-agent-codex-2026)

### CCV 项目
- 仓库: https://github.com/cggos/ccv
- 文档: https://cv.cgabc.xyz/
- PyPI: https://pypi.org/project/libccv/

---

## 8. 结论

本次实施成功将 Harness Engineering 最佳实践应用于 CCV 项目，建立了：

1. ✅ AI 代理友好的开发入口 (AGENTS.md)
2. ✅ 结构化任务追踪系统 (tasks/*.json)
3. ✅ 全面的自动化脚本 (scripts/*.sh)
4. ✅ 多配置 CI/CD 管道 (.github/workflows/)
5. ✅ 标准化构建配置 (CMakePresets.json)

这些改进使 AI 辅助开发更加可预测、可恢复、可验证，同时为人工开发者提供了便利的自动化工具。

---

**报告编制**: Claude Code  
**审核状态**: 待审核  
**版本**: 1.0
