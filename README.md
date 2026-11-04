# NexArmSim

NexArm 机械臂的 ROS 2 模型描述包，包含 URDF/Xacro、STL 网格和 RViz 配置。

当前支持在 RViz 中显示机械臂，并通过关节滑块调整姿态。项目尚未提供完整的 Gazebo 物理仿真和控制器启动配置。

## 运行环境

已在 Ubuntu 24.04、ROS 2 Jazzy 环境中验证编译、Xacro 模型解析和启动参数加载；尚未验证图形窗口运行。

需要安装 ROS 2 Jazzy，并准备以下组件：

- `colcon`
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `nav2_common`（现有 RViz 启动文件使用其中的 `ReplaceString`）

RViz 和关节滑块窗口需要在可显示图形界面的桌面终端中启动。

## 编译

在项目根目录执行。下面的路径对应当前机器，移动项目后请替换为实际路径。

```bash
cd /home/maple/Desktop/NexArmSim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select nexarm_description
```

看到 `Summary: 1 package finished` 表示编译成功。编译产物位于根目录的 `build/`、`install/` 和 `log/`，已配置 Git 忽略。

## 启动

完成编译后，在桌面终端执行：

```bash
cd /home/maple/Desktop/NexArmSim
source /opt/ros/jazzy/setup.bash
source install/setup.bash

need_compile=True ros2 launch nexarm_description display.launch.py use_sim_time:=false
```

每次打开新终端，都需要重新加载 ROS 2 环境和项目的 `install/setup.bash`。

- `need_compile=True`：让启动文件通过 ROS 包索引查找编译后的资源。必须保留该设置，否则会使用代码中写死的旧路径 `/home/ubuntu/ros2_ws/src/simulations/nexarm_description`。注意 `True` 的大小写。
- `use_sim_time:=false`：当前未启动提供 `/clock` 的仿真器，因此使用系统时间。

启动后会打开 RViz 和关节滑块窗口。拖动滑块即可改变机械臂姿态，RViz 的 `Fixed Frame` 应设置为 `base_link`。在启动终端按 `Ctrl+C` 退出。

请使用 `display.launch.py`。另一个入口 `robot_description.launch.py` 当前含有空包名和不存在的 `nexarm.xacro` 路径，尚不能直接使用。

## 常见问题

- **找不到 `nexarm_description` 包**：确认编译成功，并且在当前终端执行了 `source install/setup.bash`。
- **报错路径包含 `/home/ubuntu/ros2_ws`**：确认启动命令前设置了 `need_compile=True`。
- **提示缺少 `nav2_common` 等组件**：安装对应的 ROS 2 Jazzy 组件后，重新加载 ROS 环境。
- **RViz 不显示模型**：检查终端是否报告模型加载错误，并确认 `Fixed Frame` 为 `base_link`、启动参数为 `use_sim_time:=false`。

## 项目结构

```text
NexArmSim/
├── nexarm_description/
│   ├── launch/     # ROS 2 启动文件
│   ├── meshes/     # STL 网格
│   ├── rviz/       # RViz 显示配置
│   └── urdf/       # URDF 与 Xacro 模型
├── .gitignore
└── README.md
```

## 在水平地面画 S

新增独立的 RViz 运动学演示。先关闭原来的 `display.launch.py`，避免关节滑块与轨迹节点同时发布关节状态，然后执行：

```bash
cd /home/maple/Desktop/NexArmSim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select nexarm_description
source install/setup.bash
ros2 launch nexarm_description draw_s.launch.py
```

机械臂夹爪下方垂直安装一支 10 厘米的虚拟画笔，与夹爪伸出方向垂直，笔尖朝下，在 `base_link` 的 XY 平面（`z=0`）画出长 14 厘米、宽 7 厘米的圆弧 S，由两个相切的 270° 圆弧构成，圆弧半径 3.5 厘米，中心位于底座前方 27 厘米（X 范围 23.5–30.5 厘米）。橙色线表示画笔，蓝色线表示笔尖已走过的轨迹；默认 16 秒完成，随后停在终点。重新启动可重画。

可通过 `height:=0.01 duration:=20.0` 调整绘画平面高度（米）和时长（秒，至少 4 秒）。节点启动时先检查全路径可达性、关节限位和速度，无法完成时输出错误并停止。依赖 `python3-numpy`、`python3-scipy`、`rclpy`、`sensor_msgs`、`geometry_msgs` 和 `visualization_msgs`。

这是 RViz 中的虚拟画笔演示，尚未实现碰撞检查、物理接触或真实硬件控制。已验证完整轨迹数值检查、编译及 ROS 节点启动；无桌面测试中的 RViz 离屏渲染失败，图形效果需在桌面终端查看。

## Gazebo 中运行画 S

Gazebo Harmonic 入口会启动带地面的世界、生成机械臂和橙色虚拟笔，并通过 `ros_gz_bridge` 向 Gazebo 的关节位置控制器发送指令：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nexarm_description gazebo_draw_s.launch.py
```

请先关闭其他机械臂演示。模型生成后先留出 5 秒仿真时间移到起点，然后执行 S 轨迹，结束后保持姿态。仿真较慢时，实际等待时间会超过 21 秒。需要 `ros_gz_sim` 和 `ros_gz_bridge`。

该入口使用 Gazebo 的速度式位置跟随控制；夹爪保持闭合，画笔仅有可视几何，不计算笔尖接触力，也不会在 Gazebo 地面留下墨迹。轨迹 Marker 仍发布到 `/drawing_markers`，可用 RViz 查看。Gazebo 实际关节反馈发布在 `/nexarm/joint_states`。已验证模型生成、控制指令桥接以及反馈中笔尖沿水平面移动。

## 当前默认动作：书写 ee528

原有 `draw_s.launch.py` 和 `gazebo_draw_s.launch.py` 入口现默认书写小写 **ee528**。两个 e 使用开口圆弧，5、2 使用贝塞尔曲线，8 使用一条连续交叉曲线，一笔绕完上下两瓣，中途不抬笔。字高约 4.5 厘米，整行宽约 17 厘米，位于底座前方 25–29.5 厘米。画笔仍垂直于夹爪。

每个笔画结束后先竖直抬笔 1.5 厘米，再在空中移到下一笔起点并落笔；最后一笔结束也会抬笔。蓝色 Marker 只显示落笔线段，空中移动不留线。默认约 56 秒完成，细分曲线和逐段平滑会使总时间略长于 `duration` 参数。

```bash
ros2 launch nexarm_description draw_s.launch.py
# 抬笔高度单位为米；duration 控制基础书写时长
ros2 launch nexarm_description draw_s.launch.py lift:=0.02 duration:=60.0
```
