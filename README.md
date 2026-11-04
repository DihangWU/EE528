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
