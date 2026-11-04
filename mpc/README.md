# Linear Constrained Incremental MPC

本目录是 ROS 2 Python 包 `nexarm_mpc`。控制器位于 [linear_incremental.py](nexarm_mpc/linear_incremental.py)，电机反馈仿真位于 [simulation.py](nexarm_mpc/simulation.py)。书写节点在 30 Hz 回调中逐步执行闭环控制，RViz 显示实际关节姿态与对应笔尖轨迹；没有用平滑 Marker 代替控制。

## 编译与开关

在项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to nexarm_description
source install/setup.bash

# 关闭 MPC，电机仍有相同响应滞后
ros2 launch nexarm_description draw_s.launch.py error_deg:=0.2 motor_tau:=0.2 mpc_enabled:=false

# 开启 Linear Constrained Incremental MPC
ros2 launch nexarm_description draw_s.launch.py error_deg:=0.2 motor_tau:=0.2 mpc_enabled:=true

# 改变误差、种子与预测/控制时域
ros2 launch nexarm_description draw_s.launch.py error_deg:=0.5 error_seed:=42 mpc_enabled:=true mpc_horizon:=12 mpc_control_horizon:=8
```

默认 `mpc_enabled=false`，默认误差幅度上限 ±0.2°，默认电机响应时间常数 `motor_tau=0.2` 秒。设 `motor_tau:=0.0` 恢复原来的无滞后模型。`mpc_horizon` 是预测步数 Np，`mpc_control_horizon` 是控制增量步数 Nc，满足 `1 ≤ Nc ≤ Np ≤ 100`，默认分别为 12、8。每步为 1/30 秒。只运行一个写字节点；切换命令前关闭原演示。RViz 的“重新运行”按钮会重置控制器状态、实际轨迹、指令历史和误差序列，固定种子保证可重复对比。

## 线性模型与增量优化

现在使用一阶电机响应叠加原有到位误差：

```text
a = exp(-dt / motor_tau), dt = 1/30 秒
x[k] = a*x[k-1] + (1-a)*u[k]
y[k] = x[k] + d[k]
d[k] = 0.35*d[k-1] + 0.65*w[k]
w[k] ∈ [-error_deg, +error_deg]（换算为弧度）
```

`u` 为指令角度，`x` 为电机无随机误差的动态响应状态，`y` 为模拟实际反馈。电机收到指令立即开始逐步接近目标，并不是等待 0.2 秒后才动；一阶系统经过一个时间常数约完成阶跃变化的 63.2%，见[密歇根大学控制教程](https://ctms.engin.umich.edu/CTMS/?example=Introduction&section=SystemAnalysis)。模型使用零阶保持离散化，其基本方法可见 [SciPy cont2discrete 文档](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.cont2discrete.html)。`motor_tau=0` 时直接令 a=0，准确恢复原瞬时伺服模型。

MPC 开关前后的模拟电机使用完全相同的 a、初始状态和扰动序列。此处只加入响应滞后，没有额外纯时间延迟、漂移或接触动力学；这是可调的演示模型，不是实物电机辨识参数。现在六个舵机分别使用自己的时间常数；旧的统一设置作为批量填写和兼容入口。

控制量为 `Δu[k] = u[k] - u[k-1]`。控制器通过已发出的历史指令传播自己的名义电机状态 `x_hat`，用上一时刻实际反馈估计残差 `d_hat=y-x_hat`，再预测扰动均值 `0.35^j*d_hat`。不读取模拟电机内部状态，也不读取尚未发生的随机误差。

增广状态 `z=[x,u,d]` 的矩阵为：

```text
A = [aI, (1-a)I, 0; 0, I, 0; 0, 0, 0.35I]
B = [(1-a)I; I; 0]
C = [I, 0, I]
```

凝聚预测为 `Y = F*x_hat + (1-F)*u_previous + M*ΔU + D*d_hat`。F 为 `a^j`，D 为 `0.35^j`；M 是滞后响应矩阵，元素为 `1-a^(i-j+1)`（i≥j），否则为 0。累加矩阵 L 仍用于指令位置约束，输出与输出速度约束改为使用 M、差分矩阵与 M 的乘积。目标为：

```text
min Σ Qj * ||Yj - Rj||² + 0.002 * ||ΔU||²
```

一般阶段 Q=1，末端阶段 Q=2。控制时域之后的增量为 0，即保持最后一个预测指令。每次求解后只执行首个增量，再读反馈、重新求解。

线性硬约束包括：

- 预测指令角度位于 URDF 关节上下限内。
- 预测实际角度位于 URDF 关节上下限内。
- 每步指令增量不超过 URDF `velocity / 30`。
- 预测输出相邻步速度不超过 URDF velocity，第一步以已测量角度为基准。

J1–J6 的目标函数与约束可分离，所以分别求解六个凸二次规划，每项使用自己的预测矩阵。先计算精确的无约束最优解，只有验证它满足全部约束时才直接使用；有约束激活时，使用带解析梯度与线性约束的 SciPy SLSQP 求解同一个 QP。如果测量状态导致约束不可行或求解失败，则显式计数并退回指令位置/速率受限的参考跟踪。回退不被报告为成功的 MPC 求解。随机扰动发生后的实际速度无法由预测约束硬保证；实际关节位置仍由模拟电机按模型限位裁剪。

该实现与 [OSQP 的 MPC QP 示例](https://osqp.org/docs/examples/mpc.html) 使用相同的滚动预测与线性约束建模思路；本项目实际采用 [SciPy 的 SLSQP 接口](https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html)，没有新增 OSQP 依赖。

## 误差读数与复现实验

`/writing/pen_error_mm`（`std_msgs/msg/Float64`）发布当前实际笔尖相对同一时刻参考笔尖的三维距离，单位为毫米。每次写完时日志输出落笔采样点的笔尖三维 RMS，以及 MPC 的回退次数和求解耗时 p99。该反馈来自模拟被控对象，不是实物编码器或 Gazebo 物理反馈。

```bash
ros2 topic echo /writing/pen_error_mm
python3 mpc/compare.py --error-deg 0.2 --motor-tau 0.2 --seed 42
```

对比脚本对 MPC 关闭和开启使用完全相同的参考轨迹、电机时间常数、初始状态和扰动序列，统计仅包含落笔点。结果写到 `mpc/results/comparison.json`。当前 seed=42、误差±0.2°、motor_tau=0.2 秒：

| 指标 | MPC 关闭 | MPC 开启 |
|---|---:|---:|
| 笔尖三维 RMS / mm | 2.8622 | 0.8278 |
| 笔尖平面 RMS / mm | 2.8039 | 0.6735 |
| J1–J5 RMS / ° | 0.50976 | 0.07576 |
| 平面轮廓 RMS / mm（不计时间差） | 0.5540 | 0.4729 |

三维 RMS 改善约 71.08%，本次完整轨迹没有回退。测得平均求解约 0.1 毫秒，耗时取决于机器与约束激活情况。随机误差包含不可预测的新分量，所以 MPC 不能消除所有抖动，也不能保证每个采样点或每个种子都有相同比例的提升。

MPC 开关作用于 RViz 的模拟关节闭环。Gazebo 和实物接入需要用相应实际反馈替换模拟被控对象；当前明确拒绝 Gazebo+MPC 组合，避免把伪反馈当作真实物理反馈。原时间/关节角度 CSV 导出仍为理想参考轨迹。

## 测试

```bash
source install/setup.bash
python3 -m pytest mpc/test nexarm_description/test/test_draw_s.py nexarm_description/test/test_arrival_errors.py -q
```

测试包含主动激活的角度/速率约束、不可行回退、未来随机误差不可见、重播复位、无误差基线、一阶阶跃响应、滞后预测与独立电机滚动计算一致性，以及原字形与抬笔动作。

## RViz 面板开关

重播面板的“到位误差”输入框设置误差幅度上限，“启用 MPC”复选框设置控制器开关。两者在点击“重新运行”后一起生效，当前笔画不会中途切换。面板初始值从节点参数读取；重播时重建模拟电机、误差序列及控制器，固定种子保证对照可重复。界面全程使用异步 ROS 参数/重播调用，参数被拒绝时不自动重播。

## 调整响应滞后

```bash
# 加大响应滞后以观察效果；开关对比必须保持相同参数
ros2 launch nexarm_description draw_s.launch.py motor_tau:=0.3 error_deg:=0.2 mpc_enabled:=false
# 只观察滞后，不加随机误差
ros2 launch nexarm_description draw_s.launch.py motor_tau:=0.2 error_deg:=0.0
# 关闭滞后，保留原有到位误差
ros2 launch nexarm_description draw_s.launch.py motor_tau:=0.0 error_deg:=0.2
```

时间常数应是非负有限数，单位为秒。RViz 面板原有的误差输入与 MPC 开关继续有效，`motor_tau` 可通过 RViz 面板的“响应时间 τ”输入框或启动命令指定。也可用 `ros2 param set /draw_s motor_tau 0.3` 暂存新值，点击“重新运行”后才应用；当前动作不会中途改变。设置 `error_deg=0` 只关闭随机误差，响应滞后仍会造成跟踪误差。Gazebo 入口仍使用其原有物理控制，不额外叠加这个 RViz 电机模型。

上表为当前默认模型的实测结果；原无滞后模型的约 6.7% 改善可用 `--motor-tau 0` 单独复现。不同参数或模型不匹配时改善幅度会不同。

## 视觉对比与指标区别

RViz 实际线宽为 0.6 mm，参考灰线为 0.3 mm；MPC 关闭时实际字迹为橙红色，开启时为青蓝色。颜色区别用于识别当前模式，不是同时保留两次书写。

时间对齐 RMS 既包含轮廓变形，也包含沿轨迹方向的时间滞后。71.08% 的改善不能解释成最终字形改善 71.08%。脚本增加 `pen_xy_contour_rms_mm`：对每个实际落笔点，计算到同一笔画参考折线的 XY 最近距离，不比较采样时间。此指标默认从 0.5540 mm 降到 0.4729 mm（约 14.6%），所以静态视觉差异比时间跟踪误差差异小。

## 在面板上观察动态误差

RViz 面板显示 `/writing/pen_error_mm` 的实时三维跟踪误差和 `/writing/pen_rms_mm` 的本次累计落笔 RMS。两项是同一采样时刻的误差，因此能观察电机滞后；单独看画完后的线形容易忽略沿曲线方向的滞后。RMS 在重播后清零，抬笔采样不纳入累计值。

纯滞后可通过 `python3 mpc/compare.py --error-deg 0 --motor-tau 0.2 --output mpc/results/comparison_lag_only.json` 复现。当前结果：无 MPC 三维跟踪 RMS 2.7504 mm、XY 轮廓 RMS 0.2517 mm；有 MPC 分别为 0.0290 mm、0.0019 mm。该实验无随机新扰动且模型完全匹配，所以补偿幅度比有随机误差时更大，不代表实物必然达到此效果。

## 面板设置响应时间

“响应时间 τ”输入框单位为秒，范围 0–60 秒，显示三位小数。初始值读取当前 `motor_tau`（默认 0.2 秒）。修改后不会立即影响当前动作；点击“重新运行”时，它与随机误差、MPC 开关一起原子提交并应用。设为 0 关闭一阶响应滞后。

该输入调整的是响应时间常数，不是固定等待时间。可先用 0.1、0.2、0.5 秒作对比；时间常数越大，未补偿的响应越慢。重播时模拟电机与 MPC 的模型同时使用新时间常数，确保开关对比使用同一模型。

## 六舵机独立响应与统一设置（当前版本）

RViz 面板将原统一响应设置整合为“统一响应 τ”和 J1～J6 六项输入，单位均为秒。J6 对应夹爪开合。输入统一值后点击 **统一到六项**，会填入六个输入框；仍可以单独修改其中某项。只有点击 **重新运行**，整组设置才会同时应用到模拟电机与 MPC。没有再额外叠加一个统一滞后。

- 每项的 **归零**：该项输入设为 0。
- 统一行的 **归零**：统一值与六项响应时间一起设为 0。
- **参数全部归零**：随机误差、统一值、六项响应时间设为 0，MPC 关闭。
- 所有参数归零只修改待用输入，当前运行不变；点击“重新运行”应用。

J6 也进入响应模型与 MPC，但写字期间夹爪参考值保持夹持角度，且不向它注入随机到位误差，避免松开画笔。目标不变时，调整 J6 响应时间不会改变字迹；改变夹爪目标时，该时间常数才影响它到位的过程。原 J1–J5 随机序列保持一致。

```bash
# 兼容旧命令：六个舵机全部为 0.2 s
ros2 launch nexarm_description draw_s.launch.py motor_tau:=0.2
# 单独覆盖 J2、J3、J4，其余继承统一值
ros2 launch nexarm_description draw_s.launch.py motor_tau:=0.2 motor_tau_j2:=0.1 motor_tau_j3:=0.4 motor_tau_j4:=0.15
# 同一六舵机模型的离线开关对比
python3 mpc/compare.py --motor-taus 0.2 0.1 0.4 0.15 0.2 0.2 --output mpc/results/comparison_individual.json
```

`motor_tau_j1`～`motor_tau_j6` 默认值为 -1，表示继承 `motor_tau`；0 表示该舵机无滞后。面板每次提交显式六项值，因此之后仅通过 ROS 参数改变统一值，不会覆盖已经显式指定的单项；可用面板“统一到六项”或把单项重新设为 -1 来统一。线性增广模型中原先的标量 a 变为对角矩阵，每个关节的预测响应、Hessian 和约束矩阵独立构建。所有值必须有限且非负（继承标记 -1 除外）。
