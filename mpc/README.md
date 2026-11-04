# Linear Constrained Incremental MPC

本目录是 ROS 2 Python 包 `nexarm_mpc`。控制器位于 [linear_incremental.py](nexarm_mpc/linear_incremental.py)，电机反馈仿真位于 [simulation.py](nexarm_mpc/simulation.py)。书写节点在 30 Hz 回调中逐步执行闭环控制，RViz 显示实际关节姿态与对应笔尖轨迹；没有用平滑 Marker 代替控制。

## 编译与开关

在项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to nexarm_description
source install/setup.bash

# 关闭 MPC，保留原随机误差效果
ros2 launch nexarm_description draw_s.launch.py error_deg:=0.2 mpc_enabled:=false

# 开启 Linear Constrained Incremental MPC
ros2 launch nexarm_description draw_s.launch.py error_deg:=0.2 mpc_enabled:=true

# 改变误差、种子与预测/控制时域
ros2 launch nexarm_description draw_s.launch.py error_deg:=0.5 error_seed:=42 mpc_enabled:=true mpc_horizon:=12 mpc_control_horizon:=8
```

默认 `mpc_enabled=false`，默认误差幅度上限 ±0.2°。`mpc_horizon` 是预测步数 Np，`mpc_control_horizon` 是控制增量步数 Nc，满足 `1 ≤ Nc ≤ Np ≤ 100`，默认分别为 12、8。每步为 1/30 秒。只运行一个写字节点；切换命令前关闭原演示。RViz 的“重新运行”按钮会重置控制器状态、实际轨迹、指令历史和误差序列，固定种子保证可重复对比。

## 线性模型与增量优化

当前沿用已有的到位误差模型：

```text
y[k] = u[k] + d[k]
d[k] = 0.35*d[k-1] + 0.65*w[k]
w[k] ∈ [-error_deg, +error_deg]（换算为弧度）
```

`u` 是电机指令角度，`y` 是模拟实际反馈角度。每个关节独立产生扰动。它是理想角度伺服加到位误差模型，没有额外的电机惯性、延迟或接触动力学。误差在发出指令后由模拟被控对象加入，MPC 不读取当前尚未发生或未来的随机误差。

控制量是 `Δu[k] = u[k] - u[k-1]`。在第 k 步，控制器只通过上一时刻反馈估计 `d[k-1] = y[k-1] - u[k-1]`，并使用已知参考轨迹预览，预测未来扰动均值 `0.35^j*d[k-1]`。等价线性增广状态为 `z=[u,d]`，`A=diag(I,0.35I)`、`B=[I;0]`、`C=[I,I]`。

凝聚后的预测为 `Y = 1*u_previous + L*ΔU + D*d_previous`，其中 L 为累加矩阵，D 为相关系数的幂。目标为：

```text
min Σ Qj * ||Yj - Rj||² + 0.002 * ||ΔU||²
```

一般阶段 Q=1，末端阶段 Q=2。控制时域之后的增量为 0，即保持最后一个预测指令。每次求解后只执行首个增量，再读反馈、重新求解。

线性硬约束包括：

- 预测指令角度位于 URDF 关节上下限内。
- 预测实际角度位于 URDF 关节上下限内。
- 每步指令增量不超过 URDF `velocity / 30`。
- 预测输出相邻步速度不超过 URDF velocity，第一步以已测量角度为基准。

J1–J5 的目标函数与约束可分离，所以分别求解五个凸二次规划。先计算精确的无约束最优解，只有验证它满足全部约束时才直接使用；有约束激活时，使用带解析梯度与线性约束的 SciPy SLSQP 求解同一个 QP。如果测量状态导致约束不可行或求解失败，则显式计数并退回指令位置/速率受限的参考跟踪。回退不被报告为成功的 MPC 求解。随机扰动发生后的实际速度无法由预测约束硬保证；实际关节位置仍由模拟电机按模型限位裁剪。

该实现与 [OSQP 的 MPC QP 示例](https://osqp.org/docs/examples/mpc.html) 使用相同的滚动预测与线性约束建模思路；本项目实际采用 [SciPy 的 SLSQP 接口](https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html)，没有新增 OSQP 依赖。

## 误差读数与复现实验

`/writing/pen_error_mm`（`std_msgs/msg/Float64`）发布当前实际笔尖相对同一时刻参考笔尖的三维距离，单位为毫米。每次写完时日志输出落笔采样点的笔尖三维 RMS，以及 MPC 的回退次数和求解耗时 p99。该反馈来自模拟被控对象，不是实物编码器或 Gazebo 物理反馈。

```bash
ros2 topic echo /writing/pen_error_mm
python3 mpc/compare.py --error-deg 0.2 --seed 42
```

对比脚本对 MPC 关闭和开启使用完全相同的参考轨迹、初始状态和扰动序列，统计仅包含落笔点。结果写到 `mpc/results/comparison.json`。当前 seed=42、误差±0.2°：

| 指标 | MPC 关闭 | MPC 开启 |
|---|---:|---:|
| 笔尖三维 RMS / mm | 0.8836 | 0.8244 |
| 笔尖平面 RMS / mm | 0.7195 | 0.6704 |
| 全关节 RMS / ° | 0.08044 | 0.07530 |

三维 RMS 改善约 6.70%，本次完整轨迹没有回退。测得平均求解约 0.1 毫秒，耗时取决于机器与约束激活情况。随机误差包含不可预测的新分量，所以 MPC 不能消除所有抖动，也不能保证每个采样点或每个种子都有相同比例的提升。

MPC 开关作用于 RViz 的模拟关节闭环。Gazebo 和实物接入需要用相应实际反馈替换模拟被控对象；当前明确拒绝 Gazebo+MPC 组合，避免把伪反馈当作真实物理反馈。原时间/关节角度 CSV 导出仍为理想参考轨迹。

## 测试

```bash
source install/setup.bash
python3 -m pytest mpc/test nexarm_description/test/test_draw_s.py nexarm_description/test/test_arrival_errors.py -q
```

测试包含主动激活的角度/速率约束、不可行回退、未来随机误差不可见、重播复位、无误差基线，以及原字形与抬笔动作。

## RViz 面板开关

重播面板的“到位误差”输入框设置误差幅度上限，“启用 MPC”复选框设置控制器开关。两者在点击“重新运行”后一起生效，当前笔画不会中途切换。面板初始值从节点参数读取；重播时重建模拟电机、误差序列及控制器，固定种子保证对照可重复。界面全程使用异步 ROS 参数/重播调用，参数被拒绝时不自动重播。
