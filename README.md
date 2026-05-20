## 实验内容

本周主要学习了 Docker 容器技术与 OpenCV 图像处理基础，并结合 Python 完成了简单图像读取、显示与颜色空间转换实验。通过本周学习，进一步理解了 Docker 中“镜像（Image）”与“容器（Container）”的关系，以及容器化技术在机器人开发与 AI 环境中的重要作用。同时，还学习了 OpenCV 图像处理库的基本使用方法，为后续机器人视觉感知学习打下基础。

本周完成的主要内容如下：

- 学习 Docker 核心概念（镜像、容器）
- 学习 Docker 基础命令（pull、run、ps 等）
- 理解容器与本地目录挂载机制
- 学习 Docker `-v` 参数作用
- 使用 Docker 运行 ROS2 容器
- 学习 OpenCV 图像处理基础
- 安装 OpenCV Python 环境
- 使用 Python 读取与显示图像
- 学习颜色空间转换
- 完成基础图像处理实验
- 学习 OpenCV 在机器人视觉中的应用
- 理解图像处理在 AI 系统中的作用

通过本周实验，进一步提高了 Linux、Docker 与 Python 图像处理能力，同时对机器人视觉系统有了更加深入的认识。

---

# Docker 基础学习

本周首先学习了 Docker 的基础概念。

Docker 是一种轻量级容器化技术，可以将：

- 程序
- 运行环境
- 配置文件
- 依赖库

统一打包到独立容器中运行。

传统软件安装通常需要：

- 安装依赖
- 配置环境变量
- 解决版本兼容问题

不同电脑之间还可能因为系统环境不同而导致程序无法运行。

Docker 的出现有效解决了这些问题。

通过 Docker，可以实现：

- 环境统一
- 快速部署
- 跨平台运行
- 项目复现

因此 Docker 在：

- AI 开发
- Web 服务
- 云计算
- 机器人系统

等领域应用非常广泛。

本周重点学习了 Docker 在 ROS2 与机器人开发中的使用方法。

---

# Docker 镜像与容器

本周重点理解了：

- 镜像（Image）
- 容器（Container）

之间的区别。

## 镜像（Image）

镜像可以理解为：

- 系统模板
- 环境快照

镜像中通常包含：

- Ubuntu 系统
- Python 环境
- ROS2 软件
- OpenCV 库

等内容。

镜像本身不会运行。

---

## 容器（Container）

容器是镜像运行后的实例。

通过：

```bash
docker run

即可将镜像启动为容器。

容器具有：

独立环境
相互隔离
单独运行

等特点。

因此不同项目可以使用不同环境，而不会互相影响。

通过实验，进一步理解了：

镜像 = 模板
容器 = 正在运行的系统

这个核心概念。

Docker 基础命令学习

本周学习了 Docker 常用命令。

拉取镜像：

docker pull <镜像名>

运行容器：

docker run <镜像名>

查看运行中的容器：

docker ps

查看所有容器：

docker ps -a

停止容器：

docker stop <容器ID>

删除容器：

docker rm <容器ID>

通过这些命令，进一步掌握了 Docker 的基础使用方法。

Docker 文件挂载学习

本周重点学习了 Docker 中的目录挂载机制。

Docker 默认情况下与本地文件系统是隔离的。

如果需要让容器访问本地文件，就需要使用：

-v

参数进行目录挂载。

本次实验中使用：

docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m \
  -v "$(pwd)/:/home/ws" \
  ghcr.io/tiryoh/ros2-desktop-vnc:humble

实现：

本地目录共享
ROS2 工作空间同步
文件实时更新

通过目录挂载，可以直接在本地修改代码，并在 Docker 容器中运行。

这也是 Docker 开发中的常用方法。

ROS2 Docker 容器实验

本周继续使用 Docker 运行 ROS2 桌面环境。

运行后，可以通过浏览器访问：

http://127.0.0.1:6080/

进入 Ubuntu 图形桌面。

实验截图：

Docker 容器运行示意图

在容器内部，可以正常运行：

ROS2
Python
Linux 命令
图形化程序

通过实验，进一步理解了：

Docker 容器
Linux 系统
ROS2 环境

之间的关系。

同时也认识到 Docker 在机器人开发中的重要作用。

OpenCV 图像处理学习

本周重点学习了 OpenCV 图像处理基础。

OpenCV 是一个常用的计算机视觉库，广泛应用于：

AI 视觉
图像识别
自动驾驶
机器人视觉

等领域。

首先安装 OpenCV：

pip install opencv-python opencv-contrib-python

由于部分环境存在 numpy 版本兼容问题，因此还需要：

pip install "numpy<2"

通过这些操作，成功完成了 OpenCV 环境安装。

Python 图像读取实验

本周学习了使用 Python 读取图像。

基本代码：

import cv2

img = cv2.imread("test.jpg")

cv2.imshow("image", img)

cv2.waitKey(0)
cv2.destroyAllWindows()

通过实验，成功实现：

图像读取
图像显示
窗口控制

进一步理解了 OpenCV 的基本使用方法。

图像颜色空间转换

本周还学习了 OpenCV 中的颜色空间问题。

OpenCV 默认使用：

BGR

颜色格式。

而大多数图像系统使用：

RGB

格式。

因此需要进行颜色转换：

cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

通过实验，进一步理解了：

图像颜色空间
BGR 与 RGB 区别
图像显示异常原因

等内容。

OpenCV 在机器人中的应用

本周还学习了 OpenCV 在机器人中的应用场景。

例如：

人脸识别
目标检测
自动驾驶
路径识别
颜色识别

机器人视觉系统通常需要结合：

摄像头
AI 模型
OpenCV 图像处理

共同完成环境感知。

因此 OpenCV 是机器人视觉学习中的重要基础工具。

实验截图
OpenCV 图像处理效果

Docker 容器运行示意图

遇到的问题
问题1：Docker 容器无法启动

原因：

Docker Desktop 未运行
端口被占用

解决方法：

检查 Docker Desktop 状态
确认 6080 端口未被占用
问题2：OpenCV 导入失败

原因：

OpenCV 安装不完整
Python 环境错误

解决方法：

pip install opencv-python

重新安装 OpenCV。

问题3：numpy 版本冲突

原因：

OpenCV 与 numpy 2.x 不兼容

解决方法：

pip install "numpy<2"

降低 numpy 版本。

问题4：图像颜色显示异常

原因：

OpenCV 默认使用 BGR

解决方法：

cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

进行颜色空间转换。

学习心得

通过本周学习，进一步掌握了 Docker 容器技术的基础使用方法，并理解了镜像与容器之间的关系。

同时，通过目录挂载实验，也认识到 Docker 在开发环境管理中的便利性。

此外，在 OpenCV 图像处理实验中，成功完成了：

图像读取
图像显示
颜色空间转换

等基础操作。

通过实验，也进一步认识到机器人视觉系统在 AI 与机器人中的重要作用。

总体来看，本周不仅提升了 Docker 使用能力，也增强了 Python 图像处理与计算机视觉基础，为后续学习：

AI 视觉识别
YOLO 目标检测
机器人视觉导航
OpenCV 高级处理
深度学习视觉系统

等内容打下了良好基础。