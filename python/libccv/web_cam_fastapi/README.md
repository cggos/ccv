# Webcam Pedestrian Detection System

一个基于 FastAPI 和 ZeroMQ 的分布式实时行人检测与视频流系统。

## 项目简介

本项目将视频采集/图像处理（Streamer）与 Web 显示（WebUI）分离为两个独立的应用程序，通过 **ZeroMQ (ZMQ)** 进行局域网内的高效通信。这种解耦架构使得算法端可以专注于高性能计算，而 Web 端可以灵活部署和扩展。

### 主要功能

- **实时视频流**: 基于 OpenCV 采集摄像头数据。
- **行人检测**: 使用 HOG + SVM 算法进行端侧实时行人检测。
- **分布式架构**: 采集端 (Publisher) 与显示端 (Subscriber) 通过 ZMQ 协议解耦。
- **Web 预览**: 提供包含开始/停止控制的 Web 界面。

## 项目结构

```
web_cam_fastapi/
├── streamer/           # 采集与算法端
│   ├── main.py        # 启动脚本 (ZMQ Publisher)
│   ├── camera.py      # 摄像头驱动封装
│   └── processor.py   # (已集成到 main.py) 图像算法
├── web/                # Web 显示端
│   ├── main.py        # FastAPI 应用 (ZMQ Subscriber)
│   └── templates/     # 前端页面
└── requirements.txt    # 项目依赖
```

## 快速开始

### 1. 安装依赖

```bash
pip install -r requirements.txt
```

### 2. 运行

需要分别启动两个服务（建议在两个终端窗口中运行）：

**终端 1: 启动采集与算法服务 (Streamer)**
该服务占用端口 **5555** (TCP) 用于 ZMQ 通信。

```bash
cd streamer
python main.py
```

**终端 2: 启动 Web 显示服务 (Web)**
该服务占用端口 **8000** 用于网页访问。

```bash
cd web
python main.py
```

### 3. 访问

打开浏览器访问: `http://localhost:8000`
