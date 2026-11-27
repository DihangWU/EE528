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
colcon build --symlink-install --packages-up-to nexarm_description
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
colcon build --symlink-install --packages-up-to nexarm_description
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

### 安装高度与当前地面

当前模型通过 `world_to_base_joint` 将整个机械臂抬高 10 厘米，RViz 的固定坐标系改为 `world`，网格地面保持世界坐标 `z=0`。书写默认高度为 `base_link` 坐标中的 `height=-0.10`，即世界地面；画笔仍长 10 厘米，每笔结束抬高 1.5 厘米。

为允许 J2 向下放低，本仿真 Xacro 中 J2 上限由 1.0 改为 1.4 弧度（约 80°）；完整书写实际使用约 69°–76°。这是仿真范围设定，实物范围仍需按机械臂规格确认。上文旧版高度和 Fixed Frame 说明以本节为准。

### 当前书写设置

机械臂在正前方书写 ee528，底座安装高度 **10 厘米**。RViz 的固定坐标系是 `world`，Grid 保持世界 `z=0` 不抬升。画笔从 2.5 厘米加长为 **12.5 厘米**，默认书写高度 `height=-0.10`（相对 `base_link`），理想落笔仍位于世界地面 `z=0`。书写区在底座前方 30–34.5 厘米，每笔结束抬高 1.5 厘米，8 连续一笔写完。

笔安装在夹爪前方 4 厘米、两指之间，笔径 6 毫米。写字时夹爪开合关节保持约 0.44782 弧度，两指向内夹持画笔。手动面板的初始姿态也采用此夹持位置；“全部归零”仍按所有关节零位执行。仿真 J2 上限为 1.4 弧度（约 80°）。

当前 RViz 随机到位误差默认仍为 ±0.2°，所以蓝色实际轨迹可能略高于或低于 Grid；设 `error_deg:=0.0` 可查看精确位于 Grid 的理想轨迹。本节取代前文旧的笔长和安装高度设置。

## 手动输入关节角度

关闭写字/滑块/Gazebo 演示后，运行 `./UI/start.sh`，会打开角度控制面板与 RViz。支持 J1～J5 和夹爪角度输入、滑块、归零、停止及模型限位校验。使用说明见 [UI/README.md](UI/README.md)。

## 随机电机到位误差（RViz）

`draw_s.launch.py` 默认在 J1–J5 的目标角度上叠加有界、随采样变化的随机到位误差（每关节上限 ±0.2°），模拟电机到位不一致。不同关节独立生成扰动，相邻采样稍有相关性；实际角度会裁剪到模型关节限位。每次启动固定种子 42，便于重复对比。

```bash
# 默认误差模式
ros2 launch nexarm_description draw_s.launch.py
# 更明显的误差
ros2 launch nexarm_description draw_s.launch.py error_deg:=3.0
# 无随机误差对照（仍有电机响应滞后）
ros2 launch nexarm_description draw_s.launch.py error_deg:=0.0
# 换一组随机误差
ros2 launch nexarm_description draw_s.launch.py error_seed:=123
```

灰色细线是完整的理想字形；蓝色粗线是由带误差关节角度经过正运动学计算的笔尖实际轨迹，橙色笔和 RViz 机械臂也使用同一组带误差角度。抬笔移动仍不留线，动作结束后保持最后姿态。Z 方向偏差也真实保留在 Marker 中，因此可能出现笔尖高于或低于地面；这是运动学误差演示，没有物理接触判断。MPC 及电机响应滞后设置见下文。

该误差默认仅在上述 RViz 写字启动入口启用，角度输入 UI 和现有 CSV 理想轨迹导出不受影响。参数 `error_deg` 是误差幅度上界，单位为度，不是标准差；`error_seed` 是非负整数。

### 画笔与机械臂同步

画笔现在是 URDF 中固定在 `gripper_link` 上的 `pen_link`，与夹爪共用 RobotModel/TF 更新链路；不再通过独立的橙色 Marker 发布。轨迹 Marker 的顶点预先缓存，队列深度为 1，避免旧帧堆积。修改模型后需重启启动文件，让 robot_state_publisher 与 RViz 重新加载 robot_description。

## 在 RViz 中重新运行

写字启动配置默认加载“重新运行”面板，点击其中的 **重新运行** 按钮即可清空已画的蓝色笔迹，并从第一帧重播整行字。无需关闭 RViz，可以连续重复。固定随机种子不变，因此每次误差形状一致，便于比较。

关闭旧版启动窗口，重新加载一次环境和演示即可显示新按钮：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nexarm_description draw_s.launch.py
```

如果使用自己保存的 RViz 配置，可在 `Panels → Add New Panel` 添加 `nexarm_rviz_plugins/ReplayPanel`。按钮通过 `/restart_writing`（`std_srvs/srv/Trigger`）调用写字节点，无服务时禁用，请求期间不会阻塞界面。重新编译时使用 `colcon build --symlink-install --packages-up-to nexarm_description`，以同时构建 RViz 插件。重播从起始姿态直接开始，不规划从终点回起点的连续运动。

RViz 配置默认关闭 TF 坐标轴显示，保留 Displays 中的 TF 勾选项，可随时手动开启。

## Linear Constrained Incremental MPC 开关

算法与实验说明位于 [algorithm/mpc/README.md](algorithm/mpc/README.md)。重新编译并加载工作区后，可同时指定误差与 MPC 开关：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to nexarm_description
source install/setup.bash
ros2 launch nexarm_description draw_s.launch.py error_deg:=0.2 mpc_enabled:=true
# 对照：关闭 MPC
ros2 launch nexarm_description draw_s.launch.py error_deg:=0.2 mpc_enabled:=false
```

默认关闭 MPC，保持默认误差 ±0.2°。MPC 使用在线模拟实际关节反馈进行增量 QP 优化，并约束关节位置、指令增量和预测速度。`/writing/pen_error_mm` 发布实际笔尖误差（毫米）；重播按钮同时重置闭环状态。固定 seed=42 的完整落笔轨迹中，在默认 0.2 秒响应时间常数下，三维 RMS 从约 2.862 mm 降到 0.828 mm；随机新扰动无法被完全提前抵消。此处是 RViz 模拟反馈闭环，尚未接入真实硬件或 Gazebo 控制。

### RViz 内修改误差和 MPC

“重新运行”面板现在包含 **到位误差 ±°** 输入框和 **启用 MPC** 复选框。初次打开面板会读取启动命令中的 `error_deg`、`mpc_enabled`；例如命令行指定误差 0.5°，面板就显示 0.5°。

修改输入框或复选框后，当前动作继续使用原设置。点击 **重新运行** 后，面板先原子提交误差、响应时间和 MPC 三个参数，再调用重播服务；写字节点成功创建新仿真后，才清空字迹并从头执行。新随机误差、MPC 控制器及历史状态一起重置。误差输入 0 可关闭随机偏差；取消勾选可关闭 MPC。面板操作不需要重启 RViz。

节点的 ROS 参数作为“下次运行设置”保存，命令行 `ros2 param set` 同样仅在重播后应用。非法误差或时域设置会被拒绝；重建失败会保留当前动作。启动参数仍可用，面板修改不写回启动文件，重启进程时恢复命令行指定的设置。

### 电机响应滞后

RViz 写字默认加入一阶响应滞后，时间常数 `motor_tau=0.2` 秒。MPC 开关前后使用同一种电机模型；MPC 预测矩阵同步包含滞后。参数为 0 时恢复原来的瞬时角度响应。电机偏差和 MPC 仍可在 RViz 面板中设置，点击“重新运行”后生效。

```bash
ros2 launch nexarm_description draw_s.launch.py motor_tau:=0.2 error_deg:=0.2 mpc_enabled:=false
ros2 launch nexarm_description draw_s.launch.py motor_tau:=0.2 error_deg:=0.2 mpc_enabled:=true
```

当前同种子下完整字迹三维 RMS 误差改善约 71.1%。这是一阶仿真模型的结果，不代表实物保证；详细模型、实验条件和关闭滞后的方式见 [algorithm/mpc/README.md](algorithm/mpc/README.md)。

### 查看 MPC 的轨迹差异

实际笔迹默认线宽改为 **0.6 毫米**，理想参考灰线为 **0.3 毫米**。关闭 MPC 的实际轨迹为橙红色，开启为青蓝色。理想字形始终显示，随机误差为 0 时也能对照响应滞后效果。可放大并从上方观察，笔的实体直径仍为 6 毫米。

“约 71% 改善”指同一时刻的跟踪误差，包含沿轨迹方向的时间滞后，不能直接当作静态字形改善。对默认模型补测同笔画上的最近线段距离，平面轮廓 RMS 从约 0.554 mm 降到 0.473 mm；差异小于原来的 3 mm 线宽，原图较难看出。`algorithm/mpc/compare.py` 同时输出时间对齐误差和轮廓误差。

### 纯滞后实验与面板误差读数

“重新运行”面板增加 **实时跟踪误差** 和 **本次落笔 RMS**，单位为毫米，数据来自实际笔尖和同一时刻的参考笔尖；后一项只统计本次已执行落笔采样点，重播后清零。这两个数值均是时间对齐的三维跟踪误差，不是最终字形轮廓误差。

```bash
ros2 launch nexarm_description draw_s.launch.py error_deg:=0.0 motor_tau:=0.2 mpc_enabled:=false
```

误差输入为 0 后需点击“重新运行”才会关闭随机误差。关闭随机偏差不代表关闭电机滞后；然后可取消/勾选 MPC，分别重新运行比较面板中的读数。

纯滞后（所有关节 tau=0.2 s，无随机偏差）实验结果见 `algorithm/mpc/results/comparison_lag_only.json`：关闭 MPC 时，时间对齐笔尖三维 RMS 约 2.750 mm，但不计时间差的 XY 轮廓 RMS 仅 0.252 mm；开启 MPC 后分别约 0.029 mm、0.0019 mm。同一时间常数作用于所有关节时，主要影响到达时间，最终线形不一定明显改变。静态字形相近不表示动态误差为零。

### 面板调整电机响应时间

“重新运行”面板新增 **响应时间 τ** 输入框（秒，默认 0.2，范围 0–60）。输入新值后点击“重新运行”，才会连同误差和 MPC 设置一起生效；当前动作保持原来的响应速度。填 0 关闭电机响应滞后。此值是一阶响应时间常数，不是收到指令后完全不动的等待时间。

插件更新后需关闭旧 RViz 并重新启动一次加载新输入框；之后可直接在面板反复修改和重播，不需要重启窗口。

### 六个舵机响应时间与归零（当前面板）

原统一响应时间已整合到六项设置里：J1～J5 和 **J6 夹爪**可独立设置 τ，单位为秒。统一输入后点击“统一到六项”，也可再逐项调整；点击“重新运行”才生效。每项有归零按钮，统一行归零会清零全部响应时间，“参数全部归零”还会清零随机误差并关闭 MPC，均不立即改变当前动作。

夹爪在写字时持续夹持画笔；它的目标不变，所以调整 J6 的 τ 不会改变笔尖轨迹。第六个执行器仍进入响应模型，夹爪目标变化时会体现其独立响应速度。MPC 使用各舵机对应的时间常数。

角度输入 UI 也有逐行“归零”（只把该行输入置 0，点击移动执行）和“全部归零”（移动全部关节到模型零位）。参数面板的归零不等于机械臂姿态归零。

## PID / LQR / MPC 三种仿真

当前算法统一位于 `algorithm/`，使用方式和完整双行轨迹对比见 [algorithm/README.md](algorithm/README.md)。面板“控制优化”包含“启用 MPC”、PID、LQR 三个互斥复选框；全部不选时关闭优化。选择、误差与六个响应时间一起在“重新运行”后应用，归零和恢复也包括算法选择。

命令行可分别使用 `pid_enabled:=true`、`lqr_enabled:=true`、`mpc_enabled:=true`，每次仅开启一个。原 `mpc/` 代码已迁至 `algorithm/mpc/`，ROS 包名仍为 `nexarm_mpc`。上文单行轨迹及旧参数的数字属于历史实验，当前双行仿真请参照新说明。
