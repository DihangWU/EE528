# NexArm MATLAB 书写控制仿真

独立运行现有双行 `ee528 / WU Dihang` 轨迹上的关闭优化、PID、LQR、增量约束 MPC。三个算法文件按约定直接放在 **`algorithm/`**。控制、仿真、指标和基本三维动画均在 MATLAB 内执行；日常运行不需要 ROS、RViz、Python 或额外工具箱。

本机已使用 MATLAB R2026a 验证。首次运行请将 MATLAB 当前目录切换到本目录：

```matlab
cd('/home/maple/Desktop/EE528/matlab')
setup

% 一次比较关闭优化、PID、LQR、MPC，生成对比图和结果文件
[summary, runs] = compare_controllers;

% 只运行一个模式：'off'、'pid'、'lqr' 或 'mpc'
result = run_simulation('mpc');

% 运行后播放基础三维机械臂书写动画
result = run_simulation('mpc', struct('animate', true));
```

也可从终端批量运行：

```bash
matlab -batch "cd('/home/maple/Desktop/EE528/matlab'); compare_controllers(struct('plot',false));"
```

## 设置参数

默认值集中在 `config/default_config.m`。入口函数也支持传入部分参数，未指定项保留默认值：

```matlab
cfg = struct;
cfg.motor_tau = [0.6 0.5 0.6 0.4 0 0]; % J1～J6 时间常数，秒；也可用统一标量
cfg.error_deg = 0.2;                    % J1～J5 到位误差上界 ±°
cfg.seed = 42;
cfg.horizon = 12;                       % MPC 预测步数
cfg.control_horizon = 8;                % MPC 控制增量步数
cfg.solver = 'auto';                    % 'auto'、'dual'、'quadprog'
cfg.pid = struct('kp',0.6,'ki',0.5,'kd',0.01);
cfg.lqr = struct('state_weight',20,'command_weight',0.2);
[summary, runs] = compare_controllers(cfg);

% 纯响应滞后实验
compare_controllers(struct('error_deg',0));

% 关闭所有电机滞后和随机误差
run_simulation('off', struct('motor_tau',0,'error_deg',0));

% 只计算，关闭图形与文件输出
result = run_simulation('lqr', struct('plot',false,'save_results',false));
```

时间常数为 0 表示瞬时响应，正值表示一阶响应速度；它不是固定等待时间。J6 夹爪保持夹笔目标，默认不注入随机误差。其响应模型存在，但恒定目标下不会改变笔尖轨迹。

参考轨迹的采样周期固定为 1/30 秒。`dt` 必须与输入数据一致；入口会拒绝只修改 `dt` 而未重新采样轨迹的设置。

## 目录

```text
matlab/
├── setup.m
├── run_simulation.m
├── compare_controllers.m
├── run_tests.m
├── config/default_config.m
├── algorithm/
│   ├── PIDController.m
│   ├── LQRController.m
│   └── MPCController.m
├── +nexarm/
│   ├── +model/           # 六舵机响应与机械臂正运动学
│   ├── +trajectory/      # 读取共用参考关节轨迹、笔尖和落笔标记
│   ├── +simulation/      # 扰动生成与因果闭环循环
│   ├── +metrics/         # 跟踪误差、轮廓误差和汇总表
│   ├── +visualization/   # 二维对比图与三维连杆动画
│   └── +internal/        # 配置校验和工具箱无关的 QP 求解器
├── data/                 # 随项目提供的输入与 Python 回归基准
├── tools/export_reference.py
├── tests/test_controllers.m
└── results/              # 自动生成，Git 忽略
```

`+nexarm` 中的 `+` 是 MATLAB 包目录语法。三个控制器在普通 `algorithm/` 目录，通过 `setup` 加入路径后可直接构造。它们采用统一接口：

```matlab
cfg = default_config();
data = nexarm.trajectory.loadReference();
m = data.model;
controller = MPCController(m.lower,m.upper,m.velocity,cfg.motor_tau,cfg);
q = data.reference(1,:);
[u, info] = controller.command(q,q,data.reference(1:controller.horizon,:));
controller.reset();
```

`command` 接收上一时刻测量角、上一条指令和参考预览。关节向量输出均为行向量，预览为 `Np×6`，单位为弧度；PID/LQR 的预览为 `1×6`。`reset` 清空控制器历史、观察器和统计。

## 与 Python 版本的对应

- PID 保留参考前馈、0.8 权重的微分滤波及条件抗积分饱和。
- LQR 保留每个舵机的一阶离散模型与权重，使用标量 Riccati 方程的正根，避免依赖 Control System Toolbox。
- MPC 保留增广预测含义、扰动残差观察器、终端权重 2、增量权重 0.002、预测 12 步、控制 8 步。每个关节独立求解严格凸二次规划，仅执行首步指令。
- MPC 同时约束指令位置、预测输出位置、指令增量和预测输出速度。求解不可行或不收敛时明确计数回退，并使用位置/速率受限指令。随机扰动发生后的实际输出速度不受预测约束硬保证。
- 所有模式使用同一电机模型、参考、初始状态和扰动；关闭优化时直接发送参考。控制器只能看到上一时刻反馈，新扰动在发出当前指令后加入。

本机只安装 MATLAB 和 Simulink，未找到 `quadprog`。`solver='auto'` 在其可用时使用 `quadprog`，否则使用本目录提供的 Hildreth 对偶坐标法。内置求解器检查原始约束可行性及互补条件；超过迭代上限会报告失败并回退，不把限幅当成 QP 求解成功。它适用于本项目的小规模严格凸 QP，不是通用大型优化工具箱的替代。`quadprog` 分支仅在安装并许可 Optimization Toolbox 的环境可用。

## 参考数据及随机序列

`data/reference.mat` 提供当前 Python 规划器生成的完整 2535 点双行关节轨迹、落笔标记、参考笔尖、URDF 运动学参数和 seed=42 的单位扰动。参考角为 `N×6`，顺序为 J1～J5、夹爪。坐标使用 `base_link`，单位米；三维动画加上 0.10 m 底座安装偏移后使用世界坐标。

当前版本直接使用预计算的逆运动学参考，MATLAB 中执行正运动学及全部闭环仿真。修改字形、笔长、关节模型或轨迹时，需要重新生成数据；尚未移植 Python 的字形生成和逆运动学规划器。这样默认运行完全独立，同时能逐采样核对算法迁移。

seed=42、相关系数 0.35 时，直接缩放导出的扰动序列，保证与 Python 一致。其他种子或相关系数使用 MATLAB 私有随机流，保证 MATLAB 内四模式公平且可重复，但不保证与 Python 同种子生成同一序列，不改变调用者的全局随机状态。

在项目根目录重新生成输入及 Python 基准：

```bash
/usr/bin/python3 matlab/tools/export_reference.py
```

此步骤需要 NumPy、SciPy 和仓库中的 Python 源码，无需启动 ROS、编译 ROS 包或打开 RViz。脚本直接读取 Xacro 中已明确给出的关节参数；若将关节定义改为 Xacro 宏，需相应调整导出过程。`data/metadata.json` 保存输入源文件的 SHA-256，便于识别数据版本。

## 指标与输出

`results/comparison.csv` 和 JSON 汇总每种模式的：

- 同时刻笔尖三维、XY 跟踪 RMS（毫米）。
- 同一笔画参考线段上的 XY 最近距离 RMS（毫米），忽略时间差。
- J1～J5 角度 RMS（度）。
- 回退次数、激活 QP 数和单次控制计算 p99 耗时。

轨迹误差统计仅包含落笔采样。图中瞬时误差曲线覆盖全部运动，累计 RMS 仅在落笔时累计。三维跟踪误差包含沿轨迹的时间滞后，不能解释为最终字形轮廓误差。固定默认参数下，本次验证的三维 RMS 约为：

| 模式 | 三维 RMS / mm |
|---|---:|
| 关闭优化 | 10.4946 |
| PID | 8.7337 |
| LQR | 1.7266 |
| MPC | 0.9006 |

这些来自当前模型和数据，不代表实物性能；仓库旧 README 的历史轨迹结果可能不同。

MAT 文件包含全部指令、测量、笔尖、扰动和参考数据。PNG 显示参考/实际字迹、瞬时误差和累计 RMS；三维动画使用简化连杆和画笔，不加载 STL 外观，也不模拟物理接触。

## 验证

```matlab
run_tests
```

本机 **12 项测试全部通过**，覆盖完整轨迹与 Python 的逐采样比较、电机阶跃、零滞后、LQR 闭环稳定性、PID 抗饱和、MPC 主动约束、解析 QP 最优解、不可行回退、位置/速率限幅、历史重置、未来扰动不可见，以及落笔分段指标。

关闭优化、PID、LQR 的结果与 Python 对齐至浮点精度。Python 原版 SLSQP 使用目标函数变化容差 `ftol=1e-10`，MATLAB 内置求解器使用可行性和互补条件停止；两者在激活约束时存在数值差异，完整轨迹最大笔尖位置分量差约 0.0108 mm。

为单独验证该差异，导出脚本还保存 `mpc_tight`：保持同一 Python MPC 目标和约束，仅将独立对照运行的 SLSQP 容差收紧至 `1e-14`。MATLAB 与该高精度对照的最大测量角差约 `3.61e-7 rad`、指令角差 `4.99e-6 rad`、笔尖位置分量差 `9.44e-8 m`。测试同时核对原版和高精度基准，未修改现有 Python 控制器源码。本机验证使用 `dual` 后端，未实测缺少安装文件的 `quadprog` 后端。
