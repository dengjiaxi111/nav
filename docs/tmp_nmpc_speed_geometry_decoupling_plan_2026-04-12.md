# NMPC 参考速度与几何前视解耦方案（评审稿）

> 日期：2026-04-12  
> 适用模块：`src/my_navigation/nav_components/src/nmpc.cpp`（仅方案说明，**本稿不改代码**）

## 1. 背景与问题复盘

当前 `extractLocalReference()` 中，参考速度与几何前视长度存在结构耦合：

- 几何前视：

$$
\text{effective\_horizon} = \frac{\text{horizon\_length}}{1 + \text{horizon\_kappa\_scale}\cdot\kappa_{max}}
$$

- 采样隐含速度上限：

$$
v_{implicit} = \frac{\text{step\_dist}}{dt} = \frac{\text{effective\_horizon}}{T_{horizon}}
$$

- 速度一致性钳制（当前硬钳制语义）：

$$
v_{ref,used} = \min(v_{desired}, v_{implicit})
$$

由此带来三个工程症状：

1. `desired_velocity` 在多数工况下被 `v_implicit` 限制，主导权不足。  
2. 想要弯道明显减速（减小 `effective_horizon`）时，会连带压低直道可达速度上限。  
3. `horizon_length` 同时承担“看多远”和“跑多快”两个角色，调参互相打架。

---

## 2. 目标（同事评审用）

本轮优化目标不是“提高某个权重”，而是做**通道解耦**：

- 几何通道：`horizon_length/effective_horizon` 只负责“参考点取多远”。
- 速度通道：`desired_velocity` 及其衰减策略只负责“参考速度给多少”。
- 动力学/安全边界：由 `max_linear_vel`、加速度、终点制动包络等统一约束。

一句话：**前视长度决定路径形态，速度规划决定快慢，两者不再硬绑定。**

---

## 3. 修改思路（不改 solver 维度，最小侵入）

### 3.1 保留的部分

- 保留现有 acados 维度与状态定义；不改 `N_horizon/T_horizon` 固化事实。
- 保留 `enable_curvature_horizon_adapt` 作为几何前视压缩机制（用于“看得更近”）。
- 保留终点减速锁存与安全包络逻辑。

### 3.2 需要重构的核心点

在 `extractLocalReference()` 中，将“速度一致性硬钳制”改为“独立速度规划 + 物理安全上界”：

1. **几何轨迹生成**：继续基于 `effective_horizon` 取参考点坐标与朝向。  
2. **速度参考生成**：独立计算 `v_ref_profile(i)`，主要来源：
   - 基础巡航：`desired_velocity`
   - 曲率衰减：`enable_curvature_speed_decay`
   - 终点衰减：`goal_decel_start_dist` + `goal_crawl_speed`
3. **安全上界约束**（保留）：
   - `max_linear_vel`
   - 可选制动距离上界（避免过快）
4. **删除/弱化**：`v_ref <= effective_horizon/T_horizon` 的硬绑定。

> 备注：若担心一次性去掉导致风险，可在过渡期保留软上界（例如仅做告警或较宽松 cap），但目标架构是最终移除硬耦合。

---

## 4. 参数语义重定义建议（文档与注释需同步）

建议在 `nav_params.yaml` 与代码注释中明确：

- `horizon_length`：**纯几何前视长度**，不再表达速度上限。
- `desired_velocity`：**基础巡航速度目标**，由速度规划通道主导。
- `enable_curvature_speed_decay` 等参数：**弯道减速主通道**。

这样可以避免“我改了速度参数却不生效”的使用误解。

---

## 5. 分阶段落地计划

### Phase A（低风险，1次提交）

- 仅重排 `extractLocalReference()` 内速度参考计算顺序与注释。  
- 不新增参数，先复用现有 `curvature_decay_*`、`goal_*` 参数。  
- 目标：`desired_velocity` 在直道具备明显主导权。

### Phase B（可选增强）

- 增加“速度上界来源可观测”调试输出（topic 或 throttle log）。  
- 每周期输出：`v_desired_raw / curvature_scale / goal_scale / v_ref_final`。  
- 目标：快速定位“为什么这一段降速”。

### Phase C（若需要）

- 引入更明确的速度规划参数组（例如 `speed_profile.*`），逐步从历史参数迁移。  
- 目标：长期可维护性与参数可读性。

---

## 6. 验证计划（建议同事评审后执行）

### 6.1 场景矩阵

1. **长直道**：验证可稳定接近 `max_linear_vel`（受控制与动力学约束）。  
2. **中高曲率弯道**：验证降速幅度主要由 `curvature_decay_*` 决定。  
3. **直道→弯道→直道**：验证速度过渡连续，无异常回跳。  
4. **近终点**：验证 `goal_*` 逻辑仍能平稳收敛到 `goal_crawl_speed`。

### 6.2 关注指标

- 跟踪误差峰值（横向/航向）
- 速度曲线连续性（有无锯齿/突变）
- NMPC 求解成功率与耗时
- 安全包络触发频率（不应异常增高）

---

## 7. 风险与回滚

### 主要风险

- 去掉硬耦合后，某些工况可能出现“速度参考偏积极”。
- 若曲率衰减参数设置不当，可能弯道速度过高。

### 缓解策略

- 先启用保守 `curvature_decay_*` 初值；逐步放开。  
- 保留现有 `max_linear_vel/max_linear_accel` 和终点安全包络。  
- 通过阶段化提交，确保可快速回滚到旧逻辑。

---

## 8. 评审结论建议

建议团队确认以下三条再实施：

1. 是否同意“几何通道与速度通道必须解耦”的方向。  
2. 是否接受 Phase A 先行（最小侵入，不改 solver 维度）。  
3. 是否需要同步增加可观测性输出（Phase B）。

---

## 9. 附：一句话总结

当前问题不是参数权重大小，而是**结构耦合**；要让 `desired_velocity` 真正“有用”，需要把“看多远”和“跑多快”拆成两条独立链路。