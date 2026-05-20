````markdown
# 第 11 周实验报告：Docker 进阶与 GitHub Pages 网页部署

## 实验内容

本周主要学习了 Docker 容器的进阶操作，以及 GitHub Pages 网页部署方法。通过本周实验，进一步掌握了 Docker 容器管理、镜像保存与环境更新方法，同时学习了如何将 GitHub 作业仓库部署为网页，实现实验内容在线展示。

本周完成的主要内容如下：

1. 学习 Docker 容器管理方法
2. 掌握 Docker 镜像与容器区别
3. 学习查看、停止与删除容器
4. 学习 Docker 镜像保存（commit）
5. 在容器中安装 pybullet 与 OpenCV
6. 将修改后的容器保存为新镜像
7. 学习 GitHub Pages 网页部署
8. 将作业仓库部署为静态网页
9. 优化 README.md 页面结构
10. 学习 Markdown 网页展示方式
11. 优化实验截图与目录结构
12. 提高 GitHub 项目可读性与展示效果

通过本周实验，进一步提升了 Docker 环境管理能力，也掌握了 GitHub Pages 静态网页部署方法，为后续项目展示与作品集制作打下了基础。

---

# Docker 进阶学习

本周重点学习了 Docker 容器的进阶操作。

Docker 是一种轻量级容器化技术，可以将：

- 程序
- 运行环境
- 配置文件
- 依赖库

统一打包到独立环境中运行。

相比传统环境安装方式，Docker 具有：

- 环境统一
- 快速部署
- 易于复现
- 系统隔离

等优点。

本周进一步学习了 Docker 中：

- 镜像（Image）
- 容器（Container）

之间的关系。

---

# Docker 镜像与容器

Docker 镜像可以理解为：

- 系统模板
- 环境快照

镜像中包含：

- Ubuntu 系统
- ROS2 环境
- Python
- OpenCV
- pybullet

等软件。

镜像本身不会运行。

而容器则是镜像运行后的实例。

通过：

```bash
docker run
````

即可将镜像运行成容器。

本周实验中，还学习了：

```text
镜像 = 模板
容器 = 正在运行的环境
```

这个重要概念。

---

# Docker 容器管理

本周学习了 Docker 常见管理命令。

查看运行中的容器：

```bash
docker ps
```

查看所有容器：

```bash
docker ps -a
```

停止容器：

```bash
docker stop <容器ID>
```

删除容器：

```bash
docker rm <容器ID>
```

通过这些命令，可以方便地管理 Docker 环境。

本周实验中，还观察了不同容器状态，例如：

* Running
* Exited

进一步理解了 Docker 的运行机制。

---

# Docker 镜像保存（commit）

本周重点学习了：

```bash
docker commit
```

命令。

通常情况下，如果在容器内部安装软件：

* OpenCV
* pybullet
* Python 库

当容器删除后，环境也会丢失。

因此需要将修改后的容器重新保存为镜像。

本次实验使用：

```bash
docker commit -m "install pybullet and opencv" -a "your-name" <容器ID> my-ros2-full:v1.0
```

将当前容器保存为：

```text
my-ros2-full:v1.0
```

新镜像。

通过实验，进一步理解了 Docker 环境保存机制。

---

# OpenCV 与 pybullet 环境安装

本周在 Docker 容器内部安装了：

* OpenCV
* pybullet

等机器人与视觉相关库。

安装完成后，可以直接在容器中运行：

* OpenCV 图像处理
* PyBullet 机器人仿真

等实验。

同时还学习了：

```bash
docker images
```

查看当前镜像。

通过本部分实验，进一步掌握了 Docker 开发环境定制方法。

---

# Docker 挂载目录运行

本周继续使用目录挂载方式运行 ROS2 容器。

运行命令：

```bash
docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m \
  -v "$(pwd)/:/home/ws" \
  ghcr.io/tiryoh/ros2-desktop-vnc:humble
```

其中：

```bash
-v "$(pwd)/:/home/ws"
```

用于：

* 本地目录共享
* 文件同步
* 工作空间挂载

通过目录挂载，可以直接在本地编辑代码，并在 Docker 容器中运行。

这种方式也是 ROS2 与 AI 开发中的常用方法。

---

# GitHub Pages 网页部署

本周重点学习了 GitHub Pages 网页部署。

GitHub Pages 是 GitHub 提供的静态网页托管服务，可以直接将仓库内容部署为网页。

本次实验中，将作业仓库部署为个人实验网页。

部署步骤包括：

1. 上传 README.md
2. 提交实验截图
3. 整理目录结构
4. 开启 GitHub Pages
5. 选择部署分支

部署完成后，即可通过网页访问实验内容。

实验截图：

## GitHub Pages 网页访问效果

![GitHub Pages](屏幕截图 2026-05-14 100033.png)

通过网页，可以更加直观地展示：

* 实验内容
* 实验截图
* 学习总结
* 项目结构

提高了项目展示效果。

---

# README.md 页面优化

本周还重点优化了 README.md 页面结构。

主要包括：

* 增加标题层级
* 调整 Markdown 格式
* 插入实验图片
* 优化代码块显示
* 增加目录结构说明

通过 Markdown，可以实现：

* 标题
* 列表
* 图片
* 代码高亮

等效果。

例如：

```markdown
# 一级标题
## 二级标题
```

插入图片：

```markdown
<img src="img/demo.png" width="600">
```

通过优化 README 页面，使 GitHub 仓库更加清晰易读。

---

# GitHub Pages 部署命令

提交网页内容：

```bash
git add .
```

提交修改：

```bash
git commit -m "配置 GitHub Pages"
```

上传 GitHub：

```bash
git push
```

之后在 GitHub：

* Settings
* Pages

中开启 GitHub Pages 功能。

即可自动生成网页。

---

# 实验截图

## GitHub Pages 网页访问效果

![GitHub Pages](屏幕截图 2026-05-14 100033.png)

## Docker 容器运行与保存镜像

![Docker进阶](屏幕截图 2026-05-14 100033.png)

---

# 遇到的问题

## 问题1：Docker 容器修改后环境丢失

原因：

* 删除容器后修改不会自动保存

解决方法：

```bash
docker commit
```

保存为新镜像。

---

## 问题2：GitHub Pages 访问 404

原因：

* 仓库不是 Public
* Pages 分支设置错误

解决方法：

* 设置仓库为 Public
* 正确选择 main 分支

---

## 问题3：图片无法显示

原因：

* 图片路径错误
* 图片未提交

解决方法：

使用相对路径：

```text
img/xxx.png
```

并确认图片已上传。

---

## 问题4：README 页面格式混乱

原因：

* Markdown 语法错误
* 标题层级不规范

解决方法：

* 检查 Markdown 格式
* 调整标题结构

---

# 学习心得

通过本周学习，进一步掌握了 Docker 容器的管理方法，以及镜像保存与环境更新方式。

同时，也认识到 Docker 在机器人开发中的重要作用。通过 Docker，可以快速部署：

* ROS2
* OpenCV
* pybullet

等复杂开发环境。

此外，本周还学习了 GitHub Pages 静态网页部署方法，并成功将作业仓库部署为网页，实现实验内容在线展示。

通过 README.md 页面优化，也提高了项目整体可读性与展示效果。

总体来看，本周不仅提升了 Docker 使用能力，也增强了 GitHub 项目管理与网页部署能力，为后续学习：

* Docker Compose
* 云端部署
* Web ROS
* AI 项目展示
* 个人作品集开发

等内容打下了良好基础。

```
```
