# ROS2 环境配置 README

## 项目简介

本项目主要用于完成 ROS2（Robot Operating System 2）机器人开发环境的搭建与配置，并为后续机器人仿真、强化学习、传感器通信、运动控制以及四足机器人实验提供基础运行环境。ROS2 是目前机器人领域广泛使用的开发框架，相较于传统 ROS1，ROS2 在实时性、跨平台支持、通信效率以及分布式系统支持方面具有明显优势，因此已经被大量应用于机器人导航、自动驾驶、机械臂控制、无人机系统以及强化学习机器人训练等方向。

本实验基于 Ubuntu 22.04 系统，安装 ROS2 Humble 版本，并结合 PyBullet 仿真平台完成机器人运动控制实验。整个环境配置过程包括系统更新、ROS2 软件源添加、ROS2 安装、环境变量配置、工作空间创建、强化学习依赖安装以及机器人仿真测试等内容。完成配置后，可以正常运行 ROS2 节点通信、PyBullet 仿真以及四足机器人强化学习实验。

本 README 的目标是帮助学生快速完成 ROS2 环境搭建，并能够顺利运行 Week13 中提供的四足机器人实验代码，为后续机器人开发与强化学习研究打下基础。

---

# 一、实验环境

## 1. 操作系统

本实验主要使用：

* Ubuntu 22.04 LTS
* Windows WSL2 Ubuntu 22.04（可选）

Ubuntu 是 ROS2 官方推荐的系统环境，具有较高稳定性和良好的兼容性，适合机器人开发与科研实验。

---

## 2. ROS2 版本

本项目安装版本：

* ROS2 Humble Hawksbill

该版本属于长期支持（LTS）版本，更新稳定，适合教学与项目开发。

---

## 3. Python 环境

本实验使用：

* Python 3.10

ROS2 中大量功能与节点基于 Python 编写，因此需要保证 Python 环境正常。

---

# 二、系统更新

在安装 ROS2 之前，需要先更新 Ubuntu 软件包，避免由于依赖版本过旧导致安装失败。

执行以下命令：

```bash id="k7c2ap"
sudo apt update
sudo apt upgrade -y
```

其中：

* `apt update` 用于更新软件源列表
* `apt upgrade` 用于升级系统中的软件包

系统更新完成后，可以提高 ROS2 安装的稳定性。

---

# 三、添加 ROS2 软件源

由于 Ubuntu 默认软件源中不包含 ROS2，需要手动添加官方软件源。

## 1. 安装必要工具

```bash id="t3q9bn"
sudo apt install software-properties-common curl -y
```

这里安装了：

* software-properties-common
* curl

用于后续配置软件源与下载密钥文件。

---

## 2. 添加 ROS2 GPG 密钥

```bash id="m8f1xd"
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
-o /usr/share/keyrings/ros-archive-keyring.gpg
```

GPG 密钥用于验证 ROS2 软件包来源，保证安装安全。

---

## 3. 添加 ROS2 官方软件源

```bash id="a2k4vu"
echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

添加完成后重新更新软件源：

```bash id="w6n8pe"
sudo apt update
```

---

# 四、安装 ROS2

## 1. 安装完整版 ROS2

```bash id="u1m3zr"
sudo apt install ros-humble-desktop -y
```

desktop 版本包含：

* RViz2
* Gazebo
* rqt
* demo_nodes
* ROS2 常用工具

适合机器人学习与实验开发。

---

## 2. 安装编译工具

```bash id="d4x7ls"
sudo apt install python3-colcon-common-extensions -y
```

其中：

* colcon 是 ROS2 的工作空间编译工具
* 用于构建 ROS2 项目

---

# 五、环境变量配置

安装完成后，需要配置环境变量，否则终端无法识别 ROS2 命令。

执行：

```bash id="e9v2tb"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

然后刷新环境：

```bash id="h5r8yc"
source ~/.bashrc
```

完成后即可直接使用 `ros2` 命令。

---

# 六、ROS2 功能测试

为了验证 ROS2 是否安装成功，需要进行简单通信测试。

## 1. 启动 Talker 节点

打开第一个终端：

```bash id="q3w6zn"
ros2 run demo_nodes_cpp talker
```

该节点会不断发布字符串消息。

---

## 2. 启动 Listener 节点

打开第二个终端：

```bash id="c8k1la"
ros2 run demo_nodes_py listener
```

如果配置正确，可以看到 listener 持续接收到消息。

---

## 3. 查看 Topic

```bash id="n2u5mv"
ros2 topic list
```

会显示：

```text id="p7f9xe"
/chatter
/rosout
/parameter_events
```

说明 ROS2 Topic 通信正常。

---

# 七、创建 ROS2 工作空间

ROS2 项目通常需要在工作空间中开发。

## 1. 创建工作空间目录

```bash id="g1x4kr"
mkdir -p ~/ros2_ws/src
```

---

## 2. 进入工作空间

```bash id="j8t2qd"
cd ~/ros2_ws
```

---

## 3. 编译工作空间

```bash id="s4m7yc"
colcon build
```

编译完成后会生成：

* build/
* install/
* log/

等目录。

---

## 4. 加载工作空间环境

```bash id="l9v5pn"
source install/setup.bash
```

之后即可在工作空间中开发 ROS2 项目。

---

# 八、安装强化学习与仿真依赖

为了运行四足机器人强化学习实验，需要安装 PyBullet 与相关依赖。

## 1. 安装基础依赖

```bash id="b6u1er"
pip install pybullet numpy gymnasium
```

其中：

* pybullet 用于物理仿真
* numpy 用于数值计算
* gymnasium 用于强化学习环境

---

## 2. 安装强化学习框架

```bash id="f2q8lw"
pip install stable-baselines3 torch
```

这里：

* stable-baselines3 提供 PPO 算法
* torch 用于深度学习模型训练

---

## 3. 安装图像处理工具

```bash id="x7n3pc"
pip install opencv-python imageio matplotlib pillow
```

这些工具用于：

* 视频录制
* GIF 生成
* 图表绘制
* 图像处理

---

# 九、四足机器人实验

本项目使用 PyBullet 进行四足机器人仿真。

## 1. 方块自由落体测试

```bash id="y5m9qd"
python3 week13/demos/01_pybullet_box.py
```

运行后会弹出 PyBullet 窗口，并显示方块下落。

---

## 2. 加载四足机器人

```bash id="u4f2zk"
python3 week13/demos/02_load_laikago.py
```

系统会加载 Laikago 四足机器人模型。

---

## 3. Trot 步态测试

```bash id="e3c7ls"
python3 week13/demos/04_trot_gait.py
```

机器人会执行 Trot 步态运动。

Trot 是四足机器人中常见的对角步态，具有较好的稳定性与速度。

---

# 十、强化学习楼梯实验

本实验还包含 PPO 强化学习楼梯任务。

运行：

```bash id="k1r6mv"
python3 week13/quadruped_ppo_residual_stairs.py demo \
--task stairs \
--model week13/ppo_residual_stairs.zip \
--steps 500 \
--gui
```

实验中机器人会尝试爬上低台阶。

该实验使用：

* PPO 强化学习算法
* Residual Controller
* PyBullet 物理仿真

通过奖励函数优化机器人运动策略。

---

# 十一、常见问题

## 1. ros2 命令无法使用

可能是环境变量未加载。

解决方式：

```bash id="t8v2ep"
source /opt/ros/humble/setup.bash
```

---

## 2. GUI 无法显示

如果使用 WSL 或远程服务器，可能无法打开图形界面。

解决方式：

* 使用 X Server
* 或使用录制模式

例如：

```bash id="r4p9xm"
--record demo.mp4
```

---

## 3. Python 模块缺失

如果出现：

```text id="q6k8ab"
ModuleNotFoundError
```

需要重新安装对应模块。

例如：

```bash id="m3x7tn"
pip install pybullet
```

---

## 4. submodule 为空

如果 `week13/` 目录为空，需要初始化 submodule：

```bash id="c2w5zu"
git submodule update --init --recursive
```

---

# 十二、实验总结

通过本次 ROS2 环境配置实验，成功完成了：

* Ubuntu 开发环境搭建
* ROS2 Humble 安装
* ROS2 Topic 通信测试
* 工作空间创建
* PyBullet 仿真环境安装
* 强化学习依赖配置
* 四足机器人步态实验
* PPO 强化学习楼梯实验

本实验帮助理解了 ROS2 的基础通信机制以及机器人仿真环境的基本搭建流程。同时，通过 PyBullet 与 PPO 强化学习算法，可以进一步学习机器人运动控制与智能决策方法。

ROS2 为机器人系统提供了稳定的通信框架，而强化学习则能够帮助机器人在复杂环境中学习运动策略。未来还可以继续研究：

* Gazebo 仿真
* SLAM 建图
* MoveIt 机械臂控制
* 多机器人通信
* Isaac Gym GPU 并行强化学习
* 深度强化学习优化算法

进一步提升机器人系统的智能化水平。
