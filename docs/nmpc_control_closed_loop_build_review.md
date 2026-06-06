# NMPC 控制器修改复盘与实施清单

更新时间：2026-06-05

## 1. 范围

本文只关注 NMPC 控制器内部还需要修改什么，目标是把底盘实车速度反馈、一阶滞后模型、状态更新和调参验证闭环打通。

## 2. 当前控制器状态

当前 NMPC 模型状态是：

```text
x = [px, py, theta, v, omega, v_cmd, omega_cmd]
u = [a_cmd, alpha_cmd]
```

其中：

```text
v, omega               = 机器人实际线速度/角速度
v_cmd, omega_cmd       = 下发给底层 LQR 的速度命令状态
a_cmd, alpha_cmd       = NMPC 优化出的命令速度变化率
```

模型中已经有一阶滞后：

```text
dv/dt      = (v_cmd - v) / tau_v
domega/dt = (omega_cmd - omega) / tau_w

dv_cmd/dt      = a_cmd
domega_cmd/dt  = alpha_cmd
```

这表示底层 LQR/电机闭环不是瞬时跟随 `/cmd_vel`，而是按一阶惯性从命令速度追到实际速度。这个建模方向是对的，暂时不需要直接改成二阶模型。

## 3. 当前主要缺口

### 缺口 A：ChassisOdom 速度还没有进入 NMPC 状态

底盘已经上传：

```text
robots_msgs/msg/ChassisOdom.speed_x
robots_msgs/msg/ChassisOdom.speed_w
```

但当前 NMPC 的控制状态速度 `x0[3] / x0[4]` 还没有直接使用这两个实车速度。

现状是：

```text
/Odometry.twist        -> 当前旧实现可能参与速度状态，新方案不作为 ChassisOdom 超时 fallback
/ChassisOdom           -> 当前主要用于电容电压限幅
nmpc.chassis_odom_topic -> nav_msgs/Odometry 类型，仅调试观测，不参与控制
```

推荐目标：

```text
pose:      TF map -> base_link
velocity:  ChassisOdom.speed_x / speed_w
fallback:  NMPC 一阶预测状态 last_state_[3] / last_state_[4]
```

### 缺口 B：速度反馈比例当前关闭

当前参数：

```yaml
nmpc:
  odom_feedback_alpha: 0.0
```

当前语义是：

```text
x0_vel = alpha * measured_vel + (1 - alpha) * last_state_vel
```

`alpha = 0.0` 时，NMPC 基本不吃实测速度，速度状态更接近开环滚动。

接入 `ChassisOdom` 后，应把这个参数改成“实测速度参与比例”。建议保留旧参数名以减少改动，或新增更清晰的别名：

```yaml
nmpc:
  velocity_feedback_alpha: 0.7
```

推荐初值：

```yaml
nmpc:
  odom_feedback_alpha: 0.5
```

如果底盘速度回传稳定，再逐步调到 `0.8~1.0`。

### 缺口 C：一阶滞后 fallback 状态写回不完整

当前 `nmpc.cpp` 中已经计算了：

```cpp
v_est_next = x0[3] + v_lag_alpha * (v_cmd - x0[3]);
w_est_next = x0[4] + w_lag_alpha * (omega_cmd - x0[4]);
```

但后面写回时把实际速度状态写成了命令速度：

```cpp
last_state_[3] = v_cmd;
last_state_[4] = omega_cmd;
```

这会导致在实测速度缺失时，fallback 状态把“实际速度”误认为“命令速度”，削弱一阶滞后模型。

应该改成：

```cpp
last_state_ = x0;
last_state_[3] = v_est_next;   // 实际速度的一阶预测
last_state_[4] = w_est_next;   // 实际角速度的一阶预测
last_state_[5] = v_cmd;        // 上一拍最终下发的线速度命令
last_state_[6] = omega_cmd;    // 上一拍最终下发的角速度命令
last_control_ = u_opt;
```

如果下一拍有新的 `ChassisOdom`，它会覆盖 `x0[3]/x0[4]`；如果下一拍没有新速度，这个一阶预测值就是合理的 fallback。

### 缺口 D：速度反馈需要基础处理

`ChassisOdom` 可以作为 NMPC 实际速度反馈，但不要裸用。控制器里至少需要：

```text
1. 超时检查
2. NaN/Inf 检查
3. 离谱跳变或超限保护
4. 可选轻量低通
5. fallback 到 last_state_[3]/[4] 一阶预测速度
```

不建议做外部“超前补偿”：

```text
v_comp = v_meas + tau * dv_meas/dt
```

这类 inverse filter 会放大噪声，尤其是角速度。底盘 LQR 造成的滞后应该交给 NMPC 内部一阶模型处理。

## 4. 低通滤波建议

底盘上传给 NMPC 的速度应尽量是低延迟的实际速度估计。不要为了 NMPC 在底盘端额外加重低通，否则 NMPC 看到的是延迟后的速度，会误判底盘响应更慢。

推荐策略：

```text
底盘端：上传低延迟实车速度
导航端：保留可配置轻量低通
NMPC：用一阶滞后模型补偿执行器/LQR 响应
```

导航侧低通形式：

```text
v_f = alpha * v_meas + (1 - alpha) * v_f_prev
w_f = alpha * w_meas + (1 - alpha) * w_f_prev
```

推荐参数：

```yaml
nmpc:
  chassis_velocity_filter_alpha: 1.0
```

含义：

```text
alpha = 1.0      不滤波，延迟最小
alpha = 0.6~0.8  轻微抑制噪声
alpha < 0.5      不建议起步使用，容易引入明显额外延迟
```

实施建议：

```text
先用 alpha=1.0 跑通
如果 speed_x/speed_w 抖动明显，再试 0.8
仍明显抖动，再试 0.6
不要一开始就用强低通
```

低通应该只用于测量噪声抑制，不承担 LQR 滞后补偿功能。

## 5. 推荐代码修改方案

### 5.1 新增控制器参数

建议在 `NMPCParams` 中新增：

```cpp
std::string velocity_feedback_source = "chassis_odom";  // chassis_odom / odometry / command
double chassis_velocity_timeout_sec = 0.15;
double chassis_velocity_filter_alpha = 1.0;
double velocity_feedback_alpha = 0.5;
```

如果为了减少改动，也可以先复用：

```text
odom_feedback_alpha -> velocity_feedback_alpha
```

但文档和日志中应明确它现在代表“实测速度参与比例”，不再只表示 `/Odometry`。

### 5.2 合并 ChassisOdom 回调职责

当前 `/ChassisOdom` 已经用于电容电压。建议同一个回调中同时更新：

```text
speed_x
speed_w
capacitor_voltage
last_chassis_feedback_time
```

建议成员：

```cpp
robots_msgs::msg::ChassisOdom latest_chassis_feedback_;
rclcpp::Time latest_chassis_feedback_stamp_;
bool chassis_feedback_received_ = false;

double filtered_chassis_v_ = 0.0;
double filtered_chassis_w_ = 0.0;
bool chassis_filter_initialized_ = false;
```

如果保留 `capacitor_odom_sub_` 这个名字会造成语义混乱，建议改名为：

```cpp
chassis_feedback_sub_
```

### 5.3 增加速度读取 helper

建议新增一个内部函数：

```cpp
bool getMeasuredVelocity(double& v, double& w);
```

推荐逻辑：

```text
if source == chassis_odom and ChassisOdom fresh and valid:
    v_raw = latest_chassis_feedback_.speed_x
    w_raw = latest_chassis_feedback_.speed_w

    if filter_alpha < 1.0:
        v = alpha * v_raw + (1 - alpha) * filtered_chassis_v_
        w = alpha * w_raw + (1 - alpha) * filtered_chassis_w_
    else:
        v = v_raw
        w = w_raw

    return true

v = last_state_[3]
w = last_state_[4]
return false
```

这里的 `false` 不代表控制失败，只代表本周期没有新鲜 `ChassisOdom`，使用 NMPC 一阶预测状态 fallback。

### 5.4 computeVelocity() 中构造 x0

推荐替换为：

```cpp
double measured_v = last_state_[3];
double measured_w = last_state_[4];
const bool has_measured_velocity = getMeasuredVelocity(measured_v, measured_w);

const double alpha = std::clamp(params_.velocity_feedback_alpha, 0.0, 1.0);
x0[3] = alpha * measured_v + (1.0 - alpha) * last_state_[3];
x0[4] = alpha * measured_w + (1.0 - alpha) * last_state_[4];

x0[5] = last_state_[5];
x0[6] = last_state_[6];
```

注意：

```text
x0[3]/x0[4] 是实际速度
x0[5]/x0[6] 是上一拍命令速度
```

这两个概念不能混在一起。

### 5.5 求解后状态写回

求解成功后，应写回：

```cpp
const double tau_v = std::max(0.05, params_.vel_lag_tau);
const double tau_w = std::max(0.05, params_.omega_lag_tau);
const double v_lag_alpha = 1.0 - std::exp(-dt / tau_v);
const double w_lag_alpha = 1.0 - std::exp(-dt / tau_w);

const double v_est_next = x0[3] + v_lag_alpha * (v_cmd - x0[3]);
const double w_est_next = x0[4] + w_lag_alpha * (omega_cmd - x0[4]);

last_state_ = x0;
last_state_[3] = v_est_next;
last_state_[4] = w_est_next;
last_state_[5] = v_cmd;
last_state_[6] = omega_cmd;
last_control_ = u_opt;
```

求解失败时的衰减停车也应保持状态语义一致：

```text
last_state_[3]/[4] = 衰减后的实际速度估计
last_state_[5]/[6] = 衰减后的命令速度
```

### 5.6 更新调试观测

`/nmpc/speed_observation` 应明确发布：

```text
linear.x  = 当前最终下发 cmd_vel.linear.x
linear.y  = ChassisOdom 实测/滤波后的 speed_x
linear.z  = NMPC 预测的一拍后实际速度 v_pred_1step
angular.x = a_cmd
angular.y = vel_lag_tau
angular.z = 一拍后命令速度 v_cmd_state_pred_1step
```

如果没有新鲜 `ChassisOdom`，`linear.y` 可以填 NaN，并在日志中节流提示当前使用 fallback。

## 6. 参数初值

推荐初值：

```yaml
nmpc:
  velocity_feedback_source: "chassis_odom"
  chassis_velocity_timeout_sec: 0.15
  chassis_velocity_filter_alpha: 1.0
  velocity_feedback_alpha: 0.5

  vel_lag_tau: 0.60
  omega_lag_tau: 0.15
```

如果暂时不新增 `velocity_feedback_alpha` 参数，则先用：

```yaml
nmpc:
  odom_feedback_alpha: 0.5
```

调参顺序：

```text
1. chassis_velocity_filter_alpha = 1.0，先不滤波
2. velocity_feedback_alpha = 0.5，确认控制稳定
3. 速度反馈稳定后，把 velocity_feedback_alpha 提到 0.8
4. 如果测量抖动明显，再把 filter_alpha 调到 0.8 或 0.6
5. 最后再拟合 vel_lag_tau / omega_lag_tau
```

不要同时大幅改 `feedback_alpha`、`filter_alpha` 和 `tau`，否则很难判断问题来源。

## 7. 验证清单

### 7.1 控制器内部日志

启动后应能看到：

```text
NMPC velocity feedback: source=chassis_odom, timeout=0.15s, filter_alpha=1.00, feedback_alpha=0.50
```

如果 `ChassisOdom` 超时，应节流提示：

```text
NMPC velocity feedback stale, fallback to lag-predicted last_state velocity
```

### 7.2 观测话题

检查：

```bash
ros2 topic echo /nmpc/speed_observation
```

重点看：

```text
linear.x: NMPC 下发速度
linear.y: 底盘实测/滤波速度
linear.z: NMPC 一拍预测实际速度
angular.y: 当前 tau_v
```

期望：

```text
linear.y 不再长期为 NaN
linear.y 与底盘速度变化同步
linear.z 位于 linear.x 与 linear.y 之间，体现一阶滞后预测
```

### 7.3 ChassisOdom 超时测试

短时间断开或停止底盘速度回传时，控制器应：

```text
不崩溃
不继续无限使用过期速度
回退到 last_state_[3]/[4] 一阶预测速度
日志节流报警
```

### 7.4 低通滤波验证

先用：

```yaml
chassis_velocity_filter_alpha: 1.0
```

如果 `/nmpc/speed_observation.linear.y` 高频抖动明显，再试：

```yaml
chassis_velocity_filter_alpha: 0.8
```

仍明显抖动，再试：

```yaml
chassis_velocity_filter_alpha: 0.6
```

如果调低后出现跟踪明显变慢、弯道拖后、终点刹车变差，说明滤波引入了过多延迟，应把 `filter_alpha` 调大。

### 7.5 一阶滞后辨识

使用现有工具：

```bash
bash src/tools/tau_v_fit/run_tau_v_fit.sh 120 log/tau_v_fit step_case /nmpc/speed_observation
```

建议工况：

```text
0 -> 0.5 m/s
0 -> 1.0 m/s
1.0 -> 0 m/s
0 -> +1.0 rad/s
0 -> -1.0 rad/s
```

如果响应单调且没有明显超调/振荡，继续使用一阶模型。只有数据表现出明显二阶特征，再考虑重建二阶模型并重新导出 acados solver。

## 8. 实施顺序

### 阶段 1：速度反馈接入

```text
1. NMPC 增加 ChassisOdom 实车速度缓存
2. computeVelocity() 中优先使用 ChassisOdom.speed_x/speed_w
3. 加超时和 fallback
4. /nmpc/speed_observation.linear.y 发布实测/滤波速度
```

验收：

```text
NMPC 的 x0[3]/x0[4] 能来自 ChassisOdom
ChassisOdom 超时时能安全 fallback 到 last_state_[3]/[4]
```

### 阶段 2：状态语义修正

```text
1. 修正 last_state_[3]/[4] 写回为一阶预测实际速度
2. 保证 last_state_[5]/[6] 始终是上一拍最终命令速度
3. 求解失败、起步对齐、停车逻辑都保持同样语义
```

当前实现状态（2026-06-06）：已完成。控制器通过统一的
`updateLastStateFromCommand()` 写回 `last_state_`：

```text
last_state_[3]/[4] = 基于当前实际速度估计和最终命令速度的一阶滞后预测
last_state_[5]/[6] = 本周期最终下发的线速度/角速度命令
```

正常求解、求解失败平滑停车、连续失败停车、到达目标停车、参考轨迹异常停车、起步对齐门控均使用同一语义。

验收：

```text
实测速度缺失时，fallback 速度不会直接跳成命令速度
v/omega 与 v_cmd/omega_cmd 在日志和观测中能区分
```

### 阶段 3：低通与反馈比例调试

```text
1. filter_alpha = 1.0
2. feedback_alpha = 0.5
3. 逐步调 feedback_alpha 到 0.8~1.0
4. 如有噪声，再轻微降低 filter_alpha
```

验收：

```text
速度反馈不抖
控制不拖后
终点停车不过冲
弯道不因反馈延迟明显外切
```

### 阶段 4：tau 拟合

```text
1. 记录 /nmpc/speed_observation
2. 拟合 vel_lag_tau / omega_lag_tau
3. 更新 YAML
4. 再微调 Q_velocity / Q_omega / R_linear / R_angular
```

## 9. 控制器完成标准

满足以下条件后，可以认为 NMPC 控制器层面的速度闭环构建完成：

```text
1. NMPC 使用 ChassisOdom.speed_x/speed_w 作为实际速度反馈。
2. ChassisOdom 超时后直接 fallback 到 last_state_[3]/[4]，不使用过期速度。
3. 速度反馈有可配置轻量低通，默认不滤波。
4. 不做外部超前补偿，滞后由 NMPC 一阶模型处理。
5. last_state_[3]/[4] 表示实际速度估计。
6. last_state_[5]/[6] 表示上一拍最终命令速度。
7. /nmpc/speed_observation 能同时看到命令速度、实测速度、一拍预测速度。
8. vel_lag_tau / omega_lag_tau 经过阶跃数据校准。
```

当前最优先改的是：

```text
ChassisOdom 实车速度接入 x0[3]/x0[4]
last_state_[3]/[4] 的一阶预测写回
chassis_velocity_filter_alpha 默认 1.0 的轻量滤波接口
```
