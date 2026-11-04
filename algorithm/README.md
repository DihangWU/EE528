# 书写控制算法仿真

三个控制器分别位于 `pid/`、`lqr/`、`mpc/`，对应 ROS 包 `nexarm_pid`、`nexarm_lqr`、`nexarm_mpc`。MPC 的 Python 包名保持不变，源目录从项目根目录的 `mpc/` 移至 `algorithm/mpc/`。

## 启动及面板

在项目根目录运行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to nexarm_description
source install/setup.bash
ros2 launch nexarm_description draw_s.launch.py
```

首次打开会先计算并校验轨迹、加载机器人和面板参数；此时不推进仿真，也不产生实际笔迹。准备完成后主按钮显示“开始运行”，点击才开始；首次启动成功后按钮改为“重新运行”。待机时显示准备姿态及灰色参考轨迹，误差读数为 0。

“控制优化”一行显示“启用 MPC”、PID、LQR 三个复选框。勾选一个自动取消其他项，再点当前项可以取消；全部不选时关闭优化，但随机误差和电机滞后继续生效。选择在点击“重新运行”后应用。归零会取消三个选项。“恢复初始参数”恢复本面板首次读取的启动配置；“恢复上一次”恢复归零前或上一轮运行的配置，两者都包括误差、统一响应时间、六个舵机响应时间和控制算法。恢复只填写面板，点击“重新运行”后应用；恢复初始参数不会覆盖上一次参数的记录。更新已有安装后需要重新启动书写节点和 RViz。

也可以在启动时选择一个算法，未指定的开关默认为 false：

```bash
ros2 launch nexarm_description draw_s.launch.py pid_enabled:=true
ros2 launch nexarm_description draw_s.launch.py lqr_enabled:=true
ros2 launch nexarm_description draw_s.launch.py mpc_enabled:=true
```

后台会拒绝同时开启多个算法的设置。当前这些控制器用于 RViz 内的反馈仿真。

## 共用模型及比较条件

公共仿真位于 `mpc/nexarm_mpc/simulation.py`，按 30 Hz 推进：

```
a_j = exp(-dt / tau_j)              # tau_j=0 时 a_j=0
x[k] = a*x[k-1] + (1-a)*u[k]
y[k] = clip(x[k] + d[k], lower, upper)
d[k] = .35*d[k-1] + .65*uniform(-error, error)
```

控制器只能读取上一时刻的测量角度与已发命令，在决定新命令之后才加入新扰动。四种模式使用相同参考轨迹、初始角度、噪声种子和六个响应时间。J1–J5 加随机偏差，J6 保持夹笔；J6 也有电机模型，但恒定夹持目标下不会产生书写期间的运动滞后。

PID/LQR 对输出命令做关节位置和每步角度增量限幅；MPC 在优化内部同时处理命令及预测输出约束。未知的新随机扰动不受控制器提前支配。PID/LQR 使用当前参考点；MPC 还使用已知参考轨迹的未来点，属于算法能力差异。

默认响应时间 J1–J6 为 0.6、0.5、0.6、0.4、0、0 秒，到位误差 ±0.2°。每个算法都能使用这两类误差；优化幅度取决于参数和轨迹，不能从一种设置推导全部设置的表现。

## 完整双行轨迹对比

```bash
python3 algorithm/mpc/compare.py \
  --motor-taus 0.6 0.5 0.6 0.4 0 0 --error-deg 0.2 --seed 42 \
  --output algorithm/results/comparison_three_controllers.json
```

脚本默认比较 off、pid、lqr、mpc，也可用 `--modes off mpc` 指定子集。输出按 `off/pid/lqr/mpc` 命名各模式，不再使用旧版 `on` 键。

当前 ee528 / WU Dihang 双行轨迹共 2525 个采样点。以下均只统计落笔采样点：

| 模式 | 同时刻笔尖三维 RMS (mm) | XY 轮廓 RMS (mm) |
|---|---:|---:|
| 关闭 | 10.4841 | 3.2463 |
| PID | 8.3973 | 2.1483 |
| LQR | 1.7487 | 0.5172 |
| MPC | 0.8928 | 0.4869 |

三维跟踪误差包含时间滞后；轮廓误差取同笔画参考线段的最近距离，忽略到达时间。这些是上述一阶模型与当前固定调参的仿真结果。旧 `mpc/results/` 数据保留供历史参考，来自之前的字迹和设置。

## 验证

```bash
python3 -m pytest algorithm/mpc/test \
  nexarm_description/test/test_draw_s.py nexarm_description/test/test_arrival_errors.py -q
colcon test --packages-select nexarm_rviz_plugins --event-handlers console_direct+
```

算法测试覆盖同噪声对比、混合响应时间、命令限幅、重播状态清零、未来扰动不可见，以及 LQR 闭环稳定性；面板测试覆盖三个选项互斥、可全部关闭、延后应用、归零和恢复。
