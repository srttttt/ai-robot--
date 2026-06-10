# 📡 Mobile Vision System with ArUco Detection

基于 **Flask + OpenCV + Tailscale + ArUco** 的跨设备实时视觉采集与标记识别系统
支持手机远程摄像头访问、实时视频流传输与 ArUco 标记检测。

---

## 📑 目录

* 项目简介
* 功能特性
* 系统架构
* 环境依赖
* 项目结构
* 安装与配置
* 启动方式
* 使用方法
* 实验结果说明
* 未来优化方向

---

## 📌 项目简介

本项目构建了一个轻量级分布式视觉系统，通过手机摄像头采集图像，经由虚拟局域网络传输至本地服务器，并使用 OpenCV 对视频流进行实时 ArUco 标记检测与处理。

系统主要用于机器人视觉基础实验，验证远程图像采集、实时传输与视觉识别的一体化流程。

---

## ⚙️ 功能特性

* 📱 手机摄像头远程访问
* 🌐 基于 Tailscale 的虚拟局域网通信
* 📡 Flask 实时视频流（MJPEG）
* 🧠 OpenCV ArUco 标记识别
* 🆔 标记 ID 实时解析与绘制
* 💾 检测结果图像自动保存
* ⚡ 低延迟跨设备实时预览

---

## 🧩 系统架构

系统整体流程如下：

手机摄像头
⬇
Tailscale 虚拟局域网
⬇
Flask Web Server（WSL Ubuntu）
⬇
OpenCV 视频帧处理
⬇
ArUco 检测与标注
⬇
浏览器实时显示结果

---

## 📦 环境依赖

建议环境：

* Python 3.8+
* Ubuntu (WSL 或 Native)
* VS Code Remote WSL

### Python 依赖：

```bash
pip install flask opencv-python opencv-contrib-python numpy
```

---

## 📁 项目结构

```
project/
│
├── app.py                  # Flask 主服务
├── camera.py               # 摄像头读取模块
├── aruco_detector.py       # ArUco 检测模块
├── utils.py                # 工具函数（可选）
│
├── static/
│   └── saved_frames/       # 自动保存检测帧
│
├── templates/
│   └── index.html          # Web 页面
│
├── requirements.txt
└── README.md
```

---

## 🚀 安装与配置

### 1️⃣ 克隆项目

```bash
git clone https://github.com/yourname/your-repo.git
cd your-repo
```

---

### 2️⃣ 安装依赖

```bash
pip install -r requirements.txt
```

---

### 3️⃣ 配置 Tailscale

确保设备已加入同一虚拟网络：

* 安装 Tailscale
* 登录同一账号
* 获取 WSL / 主机 IP

官方地址：
[https://tailscale.com](https://tailscale.com)

---

## ▶️ 启动方式

### 1️⃣ 启动 Flask 服务

```bash
python app.py
```

默认运行：

```
http://0.0.0.0:5000
```

---

### 2️⃣ 手机访问

在手机浏览器输入：

```
http://<你的TailscaleIP>:5000
```

即可看到实时视频流。

---

## 📷 使用方法

1. 打开手机摄像头页面或浏览器访问服务
2. 系统自动获取视频流
3. OpenCV 实时检测 ArUco Marker
4. 标记 ID 自动显示在画面上
5. 检测到目标时自动保存截图

---

## 🧪 实验结果说明

系统成功实现以下功能：

* 手机端实时视频传输至服务器
* Flask Web 页面稳定显示 MJPEG 视频流
* OpenCV 正确识别 ArUco 标记
* 标记 ID 与位置实时可视化
* 图像保存功能正常触发
* 跨设备网络延迟较低且稳定

---

## 🔮 未来优化方向

* 📡 支持多路视频流输入
* ☁️ 云端部署（替代本地 Flask）
* 🤖 接入 ROS 机器人系统
* 🧠 加入目标跟踪（Kalman Filter）
* 📊 UI 可视化增强（Dashboard）
* ⚡ 使用 WebRTC 替代 MJPEG 提升性能

---

## 🧠 技术栈

* Flask（Web服务）
* OpenCV（图像处理）
* ArUco（视觉标记系统）
* Tailscale（虚拟局域网）
* Python（核心开发语言）

---

## 📌 备注

本项目为机器人视觉基础实验系统，适用于：

* 机器人视觉入门实验
* 网络图像传输学习
* ArUco 标记识别实践
* 跨设备实时系统搭建

