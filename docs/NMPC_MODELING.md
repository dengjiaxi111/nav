# NMPC 建模说明（导航侧）

## 1. 建模目标

在仅修改导航代码的前提下，统一三个层次：

- 外层 NMPC 的优化变量（希望保持加速度控制语义）
- 下发给底层控制器的命令（速度目标）
- 机器人真实速度响应（存在一阶滞后）

因此采用“增广状态 + 加速度控制”的结构。

## 2. 状态与控制量

- 状态：
  - \(x = [p_x, p_y, \theta, v, \omega, v_{cmd}, \omega_{cmd}]^T\)
- 控制：
  - \(u = [a_{cmd}, \alpha_{cmd}]^T\)

含义：

- \(v,\omega\)：机器人真实速度（用于轨迹跟踪与姿态控制）
- \(v_{cmd},\omega_{cmd}\)：导航下发给底层的速度命令状态
- \(a_{cmd},\alpha_{cmd}\)：速度命令变化率（优化控制量）

## 3. 连续时间动力学

\[
\begin{aligned}
\dot p_x &= v\cos\theta \\
\dot p_y &= v\sin\theta \\
\dot \theta &= \omega \\
\dot v &= \frac{v_{cmd}-v}{\tau_v} \\
\dot \omega &= \frac{\omega_{cmd}-\omega}{\tau_\omega} \\
\dot v_{cmd} &= a_{cmd} \\
\dot \omega_{cmd} &= \alpha_{cmd}
\end{aligned}
\]

其中 \(\tau_v,\tau_\omega\) 为运行时参数（下限 0.05s）。

## 4. 代价函数（EXTERNAL COST）

阶段代价由三部分组成：

- 跟踪代价：位置、航向、速度误差
- 控制代价：\(a_{cmd},\alpha_{cmd}\) 相对参考加速度的偏差
- 障碍与路径形状代价：ESDF 违反量 + contouring 偏差

并通过 `weight_scale` 支持近端/终端权重缩放。

## 5. 约束设计

- 状态约束：
  - 约束 \(v,\omega,v_{cmd},\omega_{cmd}\) 在速度上限内
- 控制约束：
  - 约束 \(a_{cmd},\alpha_{cmd}\) 在加速度上限内

这使得“命令幅值”和“命令变化率”同时可控。

## 6. 参数注入（每个 shooting node）

参数向量：

- \([x_{ref}, y_{ref}, \theta_{ref}, v_{ref}, \omega_{ref}, a_{ref}, \alpha_{ref}, d_{esdf}, weight\_scale, q_{pos}, q_{theta}, q_{vel}, r_{lin}, r_{ang}, w_{esdf}, d_{safe}, w_{contour}, \tau_v, \tau_\omega]\)

其中：

- \(a_{ref},\alpha_{ref}\) 由相邻速度参考差分得到（并按加速度限幅）
- \(\tau_v,\tau_\omega\) 由 YAML 运行时读取并注入

## 7. 工程落点

- 建模与导出：
  - `src/my_navigation/nav_components/model_ocp/model.py`
  - `src/my_navigation/nav_components/model_ocp/export_ocp.py`
- 控制器实现：
  - `src/my_navigation/nav_components/include/nav_components/nmpc.hpp`
  - `src/my_navigation/nav_components/src/nmpc.cpp`
- 生成求解器：
  - `src/my_navigation/nav_components/nmpc_solver/*`

## 8. 与旧结构的区别

- 旧结构（近期临时方案）：控制量直接为速度命令
- 新结构（当前）：控制量回到加速度，速度命令作为状态

当前结构优点：

- 更符合“外层优化加速度、底层接收速度”链路
- 更容易通过加速度约束抑制突变
- 能显式表达底层滞后，降低模型失配

## 9. 调参建议（简要）

- 先匹配时延：`vel_lag_tau`, `omega_lag_tau`
- 再调跟踪：`Q_position`, `Q_orientation`, `Q_velocity`
- 最后调平滑：`R_linear`, `R_angular`, 加速度上限

推荐顺序：先保证不过冲，再提升跟踪紧度。
