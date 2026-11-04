# LQR

实现：`nexarm_lqr/controller.py`，ROS 包：`nexarm_lqr`。

每个关节采用离散一阶模型 `x_next = a*x + b*u`，`a=exp(-dt/tau)`、`b=1-a`；tau=0 时 a=0、b=1。默认采样周期 1/30 秒，状态权重 Q=20、控制权重 R=0.2。

使用 [SciPy solve_discrete_are](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_discrete_are.html) 求解离散代数 Riccati 方程得到 P，然后计算：

```
K = (R + B.T*P*B)^-1 * B.T*P*A
u_raw = r[k] - K*(y[k-1] - r[k])
```

目标角度作为平衡点前馈，反馈使用实际测量值。每次重新运行按六个响应时间重算增益。输出命令做关节角度和速度限幅；限幅后的控制不等同于无约束 LQR 的最优解。当前是标准 LQR 跟踪调节，不带积分扩展或未来路径预览。

启动、面板操作及公平比较见 [算法说明](../README.md)。
