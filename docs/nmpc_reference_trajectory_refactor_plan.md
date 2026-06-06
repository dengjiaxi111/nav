# NMPC 参考轨迹改造方案：短前视与速度上限解耦

日期：2026-06-06  
目标模块：`src/my_navigation/nav_components/src/nmpc.cpp`，`src/my_navigation/nav_components/model_ocp/`  
状态：方案文档，暂不修改代码

## 1. 问题结论

当前 `extractLocalReference()` 已经不再使用：

```cpp
desired_v = std::min(desired_v, step_dist / dt);
```

也就是说，代码层面没有把速度参考硬限制到：

```text
v_implicit = horizon_length / T_horizon
```

但是只要 NMPC 的代价函数还在强惩罚每个时刻的绝对位置误差，`horizon_length` 仍然会形成软限速。

以当前参数为例：

```text
horizon_length = 2.55 m
T_horizon ≈ 1.5 s
v_implicit = 2.55 / 1.5 = 1.70 m/s
speed_profile.v_cruise = 2.0 m/s
Q_position = 25.0
Q_velocity = 1.0
```

参考点在 1.5 秒内只铺到前方 2.55 m，但速度参考要求 2.0 m/s。优化器如果真的跑到 2.0 m/s，预测状态会在纵向上超前这些时序位置参考，从而产生较大的 `Q_position` 代价。因此即使没有硬钳制，最终速度仍可能被位置参考软性压低。

这个问题的本质不是 `max_linear_vel` 不生效，而是：

```text
短几何前视 + 强绝对位置跟踪 = 隐式时间进度约束
```

## 2. 改造目标

我们希望同时满足：

1. `horizon_length` 可以保持较短，例如 2.4 到 2.6 m，减少切弯和远距离路径噪声影响。
2. 最大线速度由用户显式参数决定，例如 `max_linear_vel` 和 `speed_profile.v_cruise`。
3. 弯道降速由曲率速度规划决定，而不是由前视长度被动决定。
4. 终点减速由终点速度规划决定，而不是由参考点铺设长度偶然决定。
5. 不再要求 `horizon_length = desired_velocity * T_horizon`。

最终语义应变成：

```text
horizon_length: 几何前视长度，只决定参考路径形态和局部曲率观测窗口
speed_profile.v_cruise: 巡航速度目标
max_linear_vel: 最终硬速度上限
Q/contouring: 决定贴路径方式，不决定巡航速度上限
```

## 3. 当前结构复盘

### 3.1 参考轨迹生成

当前 `extractLocalReference()` 的核心流程是：

```text
effective_horizon = horizon_length
step_dist = effective_horizon / N_horizon

第 i 个参考点：
forward_dist = step_dist * i
yref[i] = path(s_nearest + forward_dist)
```

速度参考来自：

```text
base_cruise_v = speed_profile.v_cruise 或 desired_velocity
desired_v = base_cruise_v
desired_v 经过终点减速、曲率限速、原地转向、max_linear_vel 限幅
```

当前已经没有 `step_dist / dt` 的硬速度钳制。

### 3.2 代价函数结构

当前 acados external cost 中位置跟踪仍是绝对坐标误差：

```text
q_pos * ((x - x_ref)^2 + (y - y_ref)^2)
```

同时还有横向轮廓误差：

```text
contouring_weight * e_contour^2
```

其中 `e_contour` 是相对参考切线的横向偏差。这个项主要防止切弯，是正确方向。

真正造成软限速的是绝对位置误差里的纵向部分。它不仅惩罚横向偏差，也惩罚沿路径方向的超前/滞后。这样每个 shooting node 的 `x_ref/y_ref` 就不只是路径形状参考，还隐含了一个时间进度表。

## 4. 推荐总体架构

把 NMPC 参考分成三条通道：

```text
几何通道：决定路径形状、切线、曲率、ESDF 查询位置
速度通道：决定每个节点的 v_ref / omega_ref
代价通道：决定横向贴线、航向跟踪、速度跟踪、控制平滑
```

### 4.1 几何通道

几何通道继续使用短前视：

```yaml
horizon_length: 2.4 ~ 2.6
```

它只回答：

```text
我需要看前方多长的路径形状？
局部切线方向是什么？
局部曲率是多少？
ESDF 应该在哪些预测点附近线性化？
```

它不回答：

```text
机器人 1.5 秒内必须只走这么远吗？
巡航速度应该是多少？
```

### 4.2 速度通道

速度通道独立计算：

```text
v_ref = min(
  speed_profile.v_cruise,
  max_linear_vel,
  v_curve_limit,
  v_goal_limit,
  capacitor_limit_if_needed
)
```

其中曲率限速建议继续使用物理含义明确的横向加速度公式：

```text
v_curve_limit = sqrt(a_y_max / max(|kappa|, kappa_epsilon))
```

这样弯道速度由 `speed_profile.max_lateral_accel` 和曲率决定，而不是由 `effective_horizon / T_horizon` 被动决定。

### 4.3 代价通道

代价通道需要弱化或移除纵向绝对位置惩罚，让短前视不再软限制速度。

目标代价形式应从：

```text
q_pos * (dx^2 + dy^2)
```

改成：

```text
q_contour * e_contour^2
+ q_lag * e_lag^2
+ q_theta * theta_err^2
+ q_vel * (v - v_ref)^2
+ q_omega * (omega - omega_ref)^2
```

其中：

```text
e_contour = -sin(theta_ref) * dx + cos(theta_ref) * dy
e_lag     =  cos(theta_ref) * dx + sin(theta_ref) * dy
```

建议：

```text
q_contour 较大：保证不切弯、不横向偏离
q_lag 较小：只做弱纵向锚定，不把前视长度变成速度上限
q_vel 正常：让巡航速度目标真正起作用
```

## 5. 分阶段实施方案

### 阶段 0：文档与注释修正

不改变行为，只修正过期注释。

需要清理的注释包括：

```yaml
# 速度一致性补丁（代码中已实施）：desired_v = min(desired_v, step_dist/dt)
```

当前代码已经不做这个硬钳制，这句会误导调参。

建议改成：

```yaml
# horizon_length 只表示几何前视长度。
# 当前 desired_v 不再被 step_dist/dt 硬钳制；
# 但过大的 Q_position 仍可能通过时序位置参考造成软限速。
```

### 阶段 1：不重生成 solver 的过渡方案

这一阶段只改 YAML 参数和少量 C++ 注释/调试，不改 acados 模型。

目标是快速验证“软限速来自绝对位置代价”。

建议参数方向：

```yaml
horizon_length: 2.55
speed_profile:
  v_cruise: 2.0

Q_position: 3.0 ~ 8.0
Q_velocity: 1.5 ~ 3.0
contouring_weight: 15.0 ~ 30.0
```

解释：

```text
降低 Q_position：减弱纵向时序位置参考对速度的软限制
提高 contouring_weight：保持横向贴路径，减少切弯
适当提高 Q_velocity：让速度参考更有主导权
```

优点：

```text
不需要重新生成 acados solver
风险低，方便实车小速度验证
可以快速判断问题根因
```

缺点：

```text
q_pos 仍然同时包含横向和纵向误差，结构上没有彻底解耦
参数调得过激时，可能出现纵向锚定不足或终点行为变弱
```

阶段 1 是过渡方案，不是最终方案。

### 阶段 2：C++ 参考生成器结构整理

这一阶段不一定需要重生成 solver，主要是整理 `extractLocalReference()` 的职责。

建议把当前混在一起的逻辑拆成内部结构：

```cpp
struct ReferenceSample {
    double x;
    double y;
    double theta;
    double s;
    double remaining_dist;
    double kappa;
    double v_ref;
    double omega_ref;
};
```

推荐流程：

```text
1. findNearestPathPoint()
2. buildPathArcLengthCache()
3. sampleGeometryReference(horizon_length)
4. computeSpeedProfile(samples)
5. fillAcadosYref(samples)
```

这一步的目标不是改变控制行为，而是把几何采样和速度规划显式分开，避免后续继续把 `horizon_length` 当速度参数使用。

### 阶段 3：代价函数真正解耦

这是最终推荐方案，需要修改 `model_ocp/export_ocp.py` 和可能的参数注入维度，然后重新生成 acados solver。

核心修改：

```text
1. 计算 e_contour 和 e_lag
2. 用 q_contour * e_contour^2 替代大部分绝对位置跟踪
3. 保留较小 q_lag * e_lag^2 作为纵向锚定
4. 保持 q_vel * (v - v_ref)^2，让速度参考主导巡航速度
```

代价建议：

```text
tracking_cost =
    q_contour * e_contour^2
  + q_lag * e_lag^2
  + q_theta * theta_err^2
  + q_vel * (v - v_ref)^2
  + q_omega * (omega - omega_ref)^2
```

参数建议：

```yaml
Q_lag: 1.0 ~ 5.0
contouring_weight: 20.0 ~ 60.0
Q_velocity: 1.0 ~ 3.0
Q_orientation: 6.0 ~ 10.0
```

如果想减少参数维度变化，可以复用现有参数语义：

```text
Q_position -> Q_lag
contouring_weight -> Q_contour
```

但更推荐显式新增 `Q_lag`，避免 `Q_position` 名字继续误导。

阶段 3 完成后，`horizon_length = 2.55` 不应再把速度软限制到 1.7 m/s。

### 阶段 4：调试观测补齐

建议新增或扩展调试输出，至少能看到：

```text
effective_horizon
implicit_v = effective_horizon / T_horizon
v_ref_raw
v_curve_limit
v_goal_limit
v_ref_final
v_cmd
measured_v
```

验收时重点看：

```text
v_ref_final > implicit_v 时，v_cmd 是否仍能接近 v_ref_final
```

如果阶段 3 生效，那么 `implicit_v` 只应作为诊断量，不应再决定速度。

## 6. 推荐实施顺序

建议按以下顺序做：

```text
1. 先清理 YAML 和代码中过期注释。
2. 做阶段 1 参数验证：降低 Q_position，提高 contouring_weight 和 Q_velocity。
3. 上车低速验证：确认短 horizon 下速度是否更接近 v_cruise。
4. 如果阶段 1 有效果，再做阶段 2 的 C++ 结构整理。
5. 如果仍存在明显软限速，进入阶段 3，修改 acados 代价函数并重新生成 solver。
```

不要一开始就直接重生成 solver，除非已经确认参数过渡无法满足目标。

## 7. 验收标准

### 7.1 直道速度

测试配置：

```yaml
horizon_length: 2.55
speed_profile.v_cruise: 2.0
max_linear_vel: 2.0
```

期望：

```text
直道稳定段 v_cmd 能接近 2.0 m/s
实车 ChassisOdom.speed_x 能跟随上升
速度不再长期卡在 1.6 ~ 1.7 m/s 附近
```

### 7.2 弯道安全

期望：

```text
弯道速度由 v_curve_limit 降低
横向误差不明显增大
没有明显切弯
NMPC 求解成功率稳定
```

### 7.3 终点行为

期望：

```text
终点减速由 goal/s_brake 逻辑主导
不因降低 Q_position 出现末端漂移或冲过终点
如果出现末端过冲，优先调终点速度规划和制动距离，不把 horizon_length 当刹车参数
```

## 8. 风险与回滚

### 风险 1：Q_position 降低后横向贴线变差

处理：

```text
提高 contouring_weight
检查路径朝向 theta_ref 是否连续
必要时提高 Q_orientation
```

### 风险 2：速度更积极后弯道过快

处理：

```text
降低 speed_profile.max_lateral_accel
提高 kappa_epsilon 的谨慎程度
增大 curvature_window_m
```

### 风险 3：终点减速变弱

处理：

```text
打开或重调 enable_goal_speed_limit
增大 goal_slowdown_dist
降低 goal_max_slow_speed
检查 s_brake = v^2 / (2a_d) 是否足够
```

### 回滚方式

阶段 1 回滚很简单：

```text
恢复 Q_position、Q_velocity、contouring_weight 原值
```

阶段 3 如果改了 acados 代价函数，需要保留旧 solver 生成物或保留旧提交，方便整体回滚。

## 9. 最终建议

如果目标是：

```text
前视不要太长，避免切弯；
最大线速度完全由我设置；
horizon_length 不再隐式限速。
```

那么最终应该走阶段 3：

```text
用 contour/lag 代价替代绝对 x/y 位置代价。
```

阶段 1 可以先做实车验证，但它只是通过参数削弱软限速，不是结构性修复。

一句话总结：

```text
不让 horizon_length 限速的关键，不只是删掉 implicit_v 钳制；
还要让 NMPC 代价函数不再用强绝对位置跟踪来隐式规定时间进度。
```
