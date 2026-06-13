# NMPC 控制器集成总结

## 1. 当前接入状态

- `nav_server` 直接持有 `nav_components::NMPC controller_` 成员并初始化使用。
- 当前不存在 `set_controller`/`SetController` 控制器切换服务。
- `NMPC` 已接入 `LayeredMapManager`，可按配置使用 ESDF 距离代价。
- 里程计速度反馈已接入：`nmpc.odom_topic`（默认 `/Odometry`）。

## 2. NMPC 模型与求解器

- 状态维度：`x = [x, y, theta, v, omega, a_v, alpha_w, v_cmd, omega_cmd]`（9 维）。
- 控制维度：`u = [a_lin, alpha_ang]`（2 维）。
- 预测时域：`N=50`、`T=1.5s`（由 `export_ocp.py` 生成并在 solver 中固化）。
- 运行时参数维度：`NP_PARAM = 26`。
- 参数向量布局：

```text
p = [x_ref, y_ref, theta_ref, v_ref, omega_ref, a_ref, alpha_ref,
     d_esdf, x_esdf_lin, y_esdf_lin, grad_esdf_x, grad_esdf_y,
     weight_scale,
     q_pos, q_theta, q_vel, r_lin, r_ang,
     esdf_weight, esdf_safe_dist, contouring_weight,
     vel_lag_tau, omega_lag_tau, vel_lag_zeta, omega_lag_zeta, q_omega]
```

- 底盘速度响应模型为二阶滞后：
  - `a_v_dot = (v_cmd - v - 2*zeta_v*tau_v*a_v) / tau_v^2`
  - `alpha_w_dot = (omega_cmd - omega - 2*zeta_w*tau_w*alpha_w) / tau_w^2`

- 当前实现中：
  - `Q_velocity` 仅惩罚线速度误差 `(v-v_ref)^2`，不再惩罚 `(omega-omega_ref)^2`。
  - `omega_ref` 由参考航向变化率近似得到：`omega_ref ≈ d(theta_ref)/dt`（并限幅）。

## 3. 参数生效规则

### 3.1 启动读取并应用

`nmpc.cpp` 在 `initialize()` 中 `declare_parameter/get_parameter` 后：

- 立即调用 `updateNMPCParameters()` 更新速度/加速度约束到 solver。
- 在每次求解前通过 `injectEsdfParameters()` 按 stage 注入 `p`。

### 3.2 运行时注入到代价的参数

以下参数已接入 stage 级注入，确实参与代价计算：

- `nmpc.Q_position`
- `nmpc.Q_orientation`
- `nmpc.Q_velocity`
- `nmpc.R_linear`
- `nmpc.R_angular`
- `nmpc.esdf_weight`
- `nmpc.esdf_safe_dist`
- `nmpc.contouring_weight`
- `nmpc.terminal_multiplier`
- `nmpc.near_weight_multiplier`
- `nmpc.vel_lag_tau`
- `nmpc.omega_lag_tau`
- `nmpc.vel_lag_zeta`
- `nmpc.omega_lag_zeta`
- `nmpc.enable_esdf_cost`（影响是否查询/使用 ESDF 距离）

### 3.3 由 solver 固化的参数

以下内容当前不作为 YAML 运行时可调入口：

- `N_horizon`
- `T_horizon`
- QP/LM 等 acados 求解器内部选项

`N_horizon` 在代码中取 `WHEELLEG_NMPC_N`，`T_horizon` 由 solver 的 `Ts` 读取计算。

### 3.4 关于“热更新”

当前未注册参数变更回调（无 `add_on_set_parameters_callback`）。

- YAML/参数在节点启动时读取。
- 若在线 `ros2 param set` 后希望稳定生效，建议重启 `nav_server`（或整条 launch 链路）。

## 4. 跟踪与控制逻辑要点

- 到达判定当前使用 `xy_tolerance_`（位置阈值）。
- 局部参考由 `extractLocalReference()` 生成，包含：
  - 最近点搜索（含偏离过远时全局回退）
  - 按时域前向采样与插值
  - 角度连续化
  - 末段减速与横向误差触发速度缩减
- 求解失败时采用衰减式降速，连续失败达到阈值后返回失败。
- 可选发布预测轨迹话题：`nmpc/predicted_path`。

## 5. 文件对应关系

```text
src/my_navigation/nav_components/
├── include/nav_components/nmpc.hpp
├── src/nmpc.cpp
├── src/nav_server.cpp
├── model_ocp/
│   ├── model.py
│   ├── export_ocp.py
│   ├── requirements.txt
│   └── acados_ocp.json
└── nmpc_solver/
    ├── libacados_ocp_solver_wheelleg_nmpc.so
    ├── acados_solver_wheelleg_nmpc.c/.h
    └── wheelleg_nmpc_cost/, wheelleg_nmpc_model/
```

注：`src/my_navigation/nav_components/config/` 目录当前为空。

## 6. 生成与运行

```bash
# 1) 重新导出 solver（修改 model/export 后）
cd /home/nuc/navigation2026
source .venv/bin/activate
export ACADOS_SOURCE_DIR=/home/nuc/dependency/acados
export LD_LIBRARY_PATH=/home/nuc/dependency/acados/lib:$LD_LIBRARY_PATH

cd /home/nuc/navigation2026/src/my_navigation/nav_components/model_ocp
python export_ocp.py

# 2) 编译 nav_components
cd /home/nuc/navigation2026
colcon build --packages-select nav_components --symlink-install
source install/setup.bash

# 3) 启动主链路（仓库当前常用）
ros2 launch nav_bringup run.launch.py
```

## 7. 最小核查命令

```bash
ros2 node list | grep nav_server
ros2 topic list | grep -E "Odometry|cmd_vel|plan|nmpc/predicted_path"
ros2 param get /nav_server nmpc.Q_position
```

---

