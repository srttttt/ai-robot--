````markdown
# 第 8 周实验报告：Docker 与 ROS2 桌面环境

## 实验内容

本周主要学习了 Docker 容器技术以及 ROS2 桌面环境的部署方式，并成功在 Docker 容器中运行 ROS2 图形化桌面环境。通过浏览器访问远程 ROS2 桌面界面，进一步理解了容器化部署在机器人开发中的重要作用。同时，在容器内部成功运行了 turtlesim 小乌龟仿真程序，并完成了键盘控制实验。

本周完成的主要内容如下：

- 安装 Docker Desktop（Windows / Mac）
- 学习 Docker 基本概念
- 理解镜像（Image）与容器（Container）区别
- 使用 Docker 运行 ROS2 桌面环境
- 通过浏览器访问 ROS2 图形界面
- 在容器中运行 turtlesim 节点
- 使用键盘控制小乌龟运动
- 学习 Docker 与 ROS2 的结合方式
- 理解容器化部署的优势
- 了解 Docker 在机器人开发中的应用

通过本周实验，进一步理解了 Docker 在 ROS2 开发中的作用，也提高了 Linux 命令与容器环境使用能力。

---

# Docker 基础学习

本周首先学习了 Docker 的基本概念。

Docker 是一种容器化技术，可以将：

- 程序
- 依赖环境
- 配置文件
- 系统工具

一起打包到容器中运行。

相比传统软件安装方式，Docker 的优点包括：

- 环境统一
- 部署方便
- 可快速复现
- 跨平台运行
- 减少环境冲突

在机器人开发中，ROS2 环境通常依赖大量软件与库，因此 Docker 能够有效减少：

- 系统兼容问题
- 环境配置错误
- 版本冲突

等情况。

本次实验主要通过 Docker 运行 ROS2 图形化桌面环境。

---

# Docker Desktop 安装

本周在 Windows / Mac 系统中安装了 Docker Desktop。

安装过程中，需要确保：

- 已开启虚拟化
- WSL2 正常安装
- Docker Desktop 成功启动

安装完成后，可以通过：

```bash
docker --version
````

查看 Docker 是否安装成功。

同时，还学习了：

```bash
docker images
```

查看本地镜像。

以及：

```bash
docker ps
```

查看当前运行中的容器。

通过这些基础命令，进一步理解了 Docker 的基本使用方法。

---

# ROS2 桌面环境运行

本周重点实验内容是在 Docker 中运行 ROS2 桌面环境。

运行命令：

```bash
docker run ...
```

容器启动后，会自动运行：

* Ubuntu 系统
* ROS2 环境
* 浏览器远程桌面服务

随后通过浏览器访问：

```text
http://127.0.0.1:6080/
```

即可打开 ROS2 图形桌面环境。

实验截图：

## ROS2 桌面环境（浏览器访问）

![ROS2桌面](ros2_desktop.png)

在浏览器中可以看到：

* Ubuntu 桌面
* ROS2 终端
* 图形化程序窗口

通过这种方式，即使本地没有完整 ROS2 图形环境，也能够快速运行 ROS2 系统。

---

# turtlesim 小乌龟实验

进入 ROS2 桌面环境后，本周继续运行 turtlesim 小乌龟实验。

启动小乌龟：

```bash
ros2 run turtlesim turtlesim_node
```

随后启动键盘控制：

```bash
ros2 run turtlesim turtle_teleop_key
```

运行后，可以通过键盘：

* 前进
* 后退
* 左转
* 右转

控制小乌龟移动。

实验截图：

## 小乌龟仿真运行

![小乌龟](turtlesim_demo.png)

通过实验，进一步理解了：

* ROS2 节点运行
* Topic 通信
* 键盘控制
* 机器人速度控制

等内容。

同时也证明 Docker 容器中的 ROS2 环境能够正常运行图形化程序。

---

# Docker 与 ROS2 的结合

本周还学习了 Docker 与 ROS2 的结合方式。

传统 ROS2 环境配置通常需要：

* Ubuntu 系统
* Python 环境
* ROS2 安装
* 各种依赖库

配置过程较复杂。

而 Docker 可以提前打包好完整环境，用户只需要：

```bash
docker run
```

即可直接运行 ROS2。

这样能够有效减少：

* 环境配置失败
* 系统差异
* 软件版本冲突

等问题。

此外，在团队开发中，Docker 还能够：

* 统一开发环境
* 提高项目复现能力
* 方便服务器部署

因此 Docker 已经成为机器人开发中的重要工具。

---

# 浏览器远程桌面学习

本周实验中，还学习了通过浏览器访问 Linux 图形桌面。

通过：

```text
127.0.0.1:6080
```

浏览器即可直接打开容器内部桌面环境。

这种方式的优点包括：

* 不需要本地 Linux GUI
* 可以跨平台访问
* 操作更加方便
* 易于远程实验

在云服务器与机器人远程开发中，这种方式应用十分广泛。

---

# 实验运行命令

查看 Docker 版本：

```bash
docker --version
```

查看镜像：

```bash
docker images
```

查看运行中的容器：

```bash
docker ps
```

运行 ROS2 桌面环境：

```bash
docker run ...
```

浏览器访问：

```text
http://127.0.0.1:6080/
```

启动小乌龟：

```bash
ros2 run turtlesim turtlesim_node
```

键盘控制：

```bash
ros2 run turtlesim turtle_teleop_key
```

---

# 遇到的问题

## 问题1：Docker Desktop 无法启动

原因：

* WSL2 未安装
* 虚拟化未开启

解决方法：

* 安装 WSL2
* 在 BIOS 中开启虚拟化

---

## 问题2：首次下载镜像速度过慢

原因：

* 网络访问较慢
* Docker Hub 下载速度限制

解决方法：

* 使用镜像加速
* 更换网络环境

---

## 问题3：浏览器无法访问桌面

原因：

* 容器未正常运行
* 端口未映射成功

解决方法：

* 检查 docker ps
* 确认访问 `127.0.0.1:6080`

---

## 问题4：turtlesim 无法启动

原因：

* ROS2 环境未加载
* 节点未正确运行

解决方法：

```bash
source /opt/ros/humble/setup.bash
```

重新加载 ROS2 环境。

---

# 学习心得

通过本周学习，进一步掌握了 Docker 的基础使用方法，并成功在 Docker 容器中运行 ROS2 桌面环境。

同时，通过浏览器访问 Ubuntu 图形界面，也认识到容器化技术在机器人开发中的重要作用。相比传统本地安装方式，Docker 能够更加方便地统一环境、部署系统以及复现实验。

此外，在容器中运行 turtlesim 小乌龟实验，也进一步理解了：

* ROS2 节点运行机制
* 图形化程序运行方式
* Docker 与 Linux 环境关系

等内容。

总体来看，本周不仅提升了 Docker 使用能力，也增强了对 ROS2 容器化部署的理解，为后续学习：

* Docker Compose
* 多容器机器人系统
* 云端 ROS2 部署
* OpenClaw
* 机器人远程控制

等内容打下了良好基础。

```
```
