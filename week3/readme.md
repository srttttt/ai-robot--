先开一个ubuntu teminal命令行窗口
ros2 run turtlesim turtlesim_node

另开一个窗口运行
ros2 topic echo /turtle1/pose

另开一个窗口运行
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
vs code是一个代码编辑器，可以有很多工具调用，比如链接到远程服务器的工具。在wsl环境下，直接访问虚拟机ubuntu有一定困难，所以使用vs code链接到虚拟机进行文件操作/代码编辑。

1.下载安装
2.配置wsl工具
3.连接到wsl目录
