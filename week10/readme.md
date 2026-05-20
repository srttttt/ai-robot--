# ROS2 Docker + OpenCV 环境搭建与运行指南

## 📌 项目简介

本项目基于 Docker 构建 ROS2 图形化开发环境，并在容器中运行 Python + OpenCV 程序，实现图像读取与显示。
通过本项目，可以掌握 Docker 容器使用、本地目录挂载以及 Python 图像处理环境配置。

---

## 🖥️ 环境说明

* 操作系统：Windows 10 / 11
* Docker：Docker Desktop（WSL2 后端）
* 容器镜像：`ghcr.io/tiryoh/ros2-desktop-vnc:humble`
* Python版本：Python 3
* 主要库：

  * numpy
  * opencv-python
  * matplotlib

---

## 📁 项目结构

```
C:\ros2_ws
│
├── text_cv.py        # OpenCV图像处理代码
├── test.jpg          # 测试图片（需自行放入）
└── README.md
```

---

## 🚀 运行步骤

### 1️⃣ 创建工作目录

```bash
mkdir C:\ros2_ws
cd C:\ros2_ws
```

---

### 2️⃣ 启动 Docker 容器

```bash
docker run -p 6080:80 ^
--security-opt seccomp=unconfined ^
--shm-size=512m ^
-v %cd%:/home/ws ^
ghcr.io/tiryoh/ros2-desktop-vnc:humble
```

---

### 3️⃣ 打开图形界面

浏览器访问：

```
http://localhost:6080
```

默认账户：

* 用户名：ubuntu
* 密码：ubuntu

---

### 4️⃣ 进入工作目录

在容器终端执行：

```bash
cd /home/ws
ls
```

---

### 5️⃣ 安装依赖（解决版本问题）

```bash
pip3 install "numpy<2"
pip3 install opencv-python matplotlib
```

---

### 6️⃣ 运行程序

```bash
python3 text_cv.py
```

---

## ⚠️ 常见问题

### ❗ 1. Docker 无法连接

```
failed to connect to the docker API
```

👉 解决方法：

* 启动 Docker Desktop
* 确认状态为 "Docker is running"

---

### ❗ 2. 找不到文件

```
No such file or directory
```

👉 解决方法：

* 确认文件放在 `C:\ros2_ws`
* 容器中使用 `/home/ws`

---

### ❗ 3. NumPy 与 OpenCV 冲突

```
numpy.core.multiarray failed to import
```

👉 解决方法：

```bash
pip3 install "numpy<2"
```

---

### ❗ 4. 图片读取失败

```
!_src.empty()
```

👉 原因：

* 图片路径错误
* 图片不存在

👉 建议写法：

```python
img = cv2.imread('/home/ws/test.jpg')
```

---

## 🧠 项目总结

通过本项目，完成了以下内容：

* ✅ 使用 Docker 构建 ROS2 图形环境
* ✅ 实现 Windows 与容器目录挂载
* ✅ 配置 Python OpenCV 运行环境
* ✅ 解决依赖冲突问题（NumPy 与 OpenCV）
* ✅ 成功运行图像处理程序

---

## 📌 后续拓展

* 使用 ROS2 订阅摄像头图像
* 在 RViz 中显示处理结果
* 实现实时目标检测

