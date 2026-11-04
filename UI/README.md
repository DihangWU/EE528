# 关节角度控制面板

先关闭 `draw_s.launch.py`、`display.launch.py` 或 Gazebo 演示，避免多个程序同时发布关节状态。

首次使用或修改启动文件后，在项目根目录编译：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to nexarm_description
```

启动面板、机械臂 TF 和 RViz：

```bash
./UI/start.sh
```

也可加载 ROS 和工作区环境后执行：

```bash
ros2 launch nexarm_description angle_panel.launch.py
```

在 J1～J5 和夹爪行输入角度，点击“移动到指定角度”或按 Enter；模型用约 1.5 秒平滑移动到目标并保持。拖动滑块后松开鼠标也会执行整组目标角度。“全部归零”回到模型零位，“停止移动”保持当前姿态。关闭面板会一并退出本次启动的 RViz 和 TF 节点。

角度单位为度，以 URDF 关节零位为基准，不是 0–4095 舵机编码。允许范围读取自当前 Xacro 模型；任一输入无效或越界时，整组目标都不会执行。夹爪平移随模型中的开合关系联动。

面板只发布 `/joint_states` 供 RViz 显示，不控制实物或 Gazebo 控制器。如需使用已经打开的 RViz，可仅启动面板（需已有 robot_state_publisher）：

```bash
/usr/bin/python3 UI/angle_panel.py
```

依赖：ROS 2 Jazzy、当前工作区、`python3-tk`、`xacro` 和桌面图形环境。

每个关节行新增 **归零**，只把该行输入和滑块置 0，不立即执行；点击“移动到指定角度”应用。原 **全部归零** 按钮仍会执行整组关节回零。
