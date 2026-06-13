# NMPC 改为二阶滞后模型说明

日期：2026-06-13

范围：说明为什么当前一阶滞后模型在实车速度过冲/回摆时不够用，以及如何把 NMPC 模型改成二阶滞后模型。

## 1. 结论

如果实车表现为：

```text
速度命令发出后，实际速度不是单调追上，而是超过目标速度，随后回摆或振荡
```

那么把当前 NMPC 的一阶滞后改成二阶滞后是有意义的。

原因是：一阶滞后只能描述“慢慢追上”，不能描述“冲过头再回来”。过冲和回摆是典型二阶系统特征，尤其当底盘内环是 LQR、机械系统本身又有惯性时，实际闭环响应经常更接近二阶低通系统。

但需要注意：二阶滞后解决的是“底盘动态响应形状不匹配”，不是纯通信延迟。如果命令发出后先完全没反应，过一段固定时间才开始响应，那还需要额外考虑输入延迟。

## 2. 当前模型的问题

当前 NMPC 模型状态为：

```text
x = [px, py, theta, v, omega, v_cmd, omega_cmd]
u = [a_cmd, alpha_cmd]
```

其中：

```text
v         实际线速度
omega     实际角速度
v_cmd     NMPC 下发给底盘的线速度命令状态
omega_cmd NMPC 下发给底盘的角速度命令状态
a_cmd     NMPC 输出的线速度命令变化率
alpha_cmd NMPC 输出的角速度命令变化率
```

当前一阶滞后为：

```text
v_dot     = (v_cmd - v) / tau_v
omega_dot = (omega_cmd - omega) / tau_w
```

它隐含的假设是：

```text
只要 v_cmd 固定，v 会单调、平滑地追上 v_cmd，不会过冲。
```

如果实车速度会过冲或回摆，这个假设就错了。NMPC 会低估底盘惯性和回摆趋势，导致预测速度和真实速度对不上，进而影响路径跟踪和速度命令生成。

## 3. 二阶滞后的直观理解

一阶模型可以理解为：

```text
速度命令 -> 实际速度
```

二阶模型可以理解为：

```text
速度命令 -> 实际加速度 -> 实际速度
```

也就是 NMPC 不只知道“当前速度是多少”，还知道“速度正在怎么变化”。

新增两个状态：

```text
a_v       实际线加速度状态，也就是 v_dot
alpha_w   实际角加速度状态，也就是 omega_dot
```

这两个量不是 NMPC 直接下发到底盘的命令，而是 NMPC 内部用来模拟底盘响应的状态。

容易混淆的地方：

```text
a_cmd   是 NMPC 对 v_cmd 的改变速度，属于输入命令侧
a_v     是底盘真实速度 v 的变化速度，属于底盘响应侧
```

所以它们不是同一个东西。

## 4. 建议的新状态

建议把状态从 7 维改为 9 维：

```text
x = [px, py, theta, v, omega, a_v, alpha_w, v_cmd, omega_cmd]
```

控制量保持 2 维不变：

```text
u = [a_cmd, alpha_cmd]
```

各状态含义：

```text
px, py       机器人位置
theta        机器人航向角
v            实际线速度
omega        实际角速度
a_v          实际线加速度，描述 v 当前正在如何变化
alpha_w      实际角加速度，描述 omega 当前正在如何变化
v_cmd        下发给底盘的线速度命令状态
omega_cmd    下发给底盘的角速度命令状态
```

## 5. 建议的二阶滞后方程

线速度通道：

```text
tau_v^2 * v_ddot + 2 * zeta_v * tau_v * v_dot + v = v_cmd
```

角速度通道：

```text
tau_w^2 * omega_ddot + 2 * zeta_w * tau_w * omega_dot + omega = omega_cmd
```

写成一阶状态空间形式：

```text
px_dot        = v * cos(theta)
py_dot        = v * sin(theta)
theta_dot     = omega

v_dot         = a_v
omega_dot     = alpha_w

a_v_dot       = (v_cmd - v - 2 * zeta_v * tau_v * a_v) / (tau_v * tau_v)
alpha_w_dot   = (omega_cmd - omega - 2 * zeta_w * tau_w * alpha_w) / (tau_w * tau_w)

v_cmd_dot     = a_cmd
omega_cmd_dot = alpha_cmd
```

新增参数：

```text
zeta_v    线速度二阶响应阻尼比
zeta_w    角速度二阶响应阻尼比
```

参数含义：

```text
tau 小：响应更快
tau 大：响应更慢

zeta < 1.0：欠阻尼，可能过冲，响应更猛
zeta = 1.0：临界阻尼，通常先从这里开始
zeta > 1.0：过阻尼，更稳但更慢
```

如果实车有过冲，通常说明真实系统的等效 `zeta` 小于 1。建模时不一定一开始就设置成小于 1，建议先用 `zeta=1.0` 保守验证，再根据实车曲线逐步调整到 `0.7~1.0`。

## 6. 需要修改的代码位置

### 6.1 `model_ocp/model.py`

当前文件：

```text
src/my_navigation/nav_components/model_ocp/model.py
```

需要做的修改：

1. 状态从 7 维扩展到 9 维。
2. 增加 `a_v` 和 `alpha_w` 两个状态。
3. 增加 `vel_lag_zeta` 和 `omega_lag_zeta` 两个运行时参数。
4. 把 `dynamics()` 中的一阶滞后改成二阶滞后。

建议状态顺序：

```text
[x, y, theta, v, omega, a_v, alpha_w, v_cmd, omega_cmd]
```

建议原因：保留 `v/omega` 在原来的 3、4 号索引，减少代价函数和速度反馈部分的改动；把新增加速度状态放在后面；把命令状态挪到 7、8 号索引。

### 6.2 `model_ocp/export_ocp.py`

当前文件：

```text
src/my_navigation/nav_components/model_ocp/export_ocp.py
```

需要做的修改：

1. `nx` 会从 7 自动变成 9。
2. 参数默认值增加两个阻尼比参数。
3. `np` 会从 24 变成 26。
4. 状态约束索引需要从：

```python
ocp.constraints.idxbx = np.array([3, 4, 5, 6])
```

改为：

```python
ocp.constraints.idxbx = np.array([3, 4, 7, 8])
```

因为新的 `v_cmd/omega_cmd` 已经从 5、6 号索引移动到 7、8 号索引。

是否约束 `a_v/alpha_w` 可以先不做。第一版只约束真实速度和命令速度，避免约束过多导致求解更难。

### 6.3 `include/nav_components/nmpc.hpp`

当前文件：

```text
src/my_navigation/nav_components/include/nav_components/nmpc.hpp
```

需要做的修改：

1. 注释里的状态布局改成 9 维。
2. `last_state_` 注释改成：

```text
[x, y, theta, v, omega, a_v, alpha_w, v_cmd, omega_cmd]
```

3. `predicted_stage1_state_` 从 7 维改成 9 维。
4. `NP_PARAM` 从 24 改成 26。
5. 参数结构体增加：

```cpp
double vel_lag_zeta = 1.0;
double omega_lag_zeta = 1.0;
```

### 6.4 `src/nmpc.cpp`

当前文件：

```text
src/my_navigation/nav_components/src/nmpc.cpp
```

需要修改所有依赖状态索引的位置。

初始化：

```cpp
last_state_.resize(9, 0.0);
```

构造 `x0`：

```text
x0[0] = px
x0[1] = py
x0[2] = theta
x0[3] = measured_v
x0[4] = measured_w
x0[5] = last_state_[5]  // a_v
x0[6] = last_state_[6]  // alpha_w
x0[7] = last_state_[7]  // v_cmd
x0[8] = last_state_[8]  // omega_cmd
```

根据优化输出更新命令：

```cpp
double v_cmd = x0[7] + a_cmd * dt;
double omega_cmd = x0[8] + alpha_cmd * dt;
```

求解器初始状态数组：

```cpp
double x0_array[9] = {
    x0[0], x0[1], x0[2], x0[3], x0[4],
    x0[5], x0[6], x0[7], x0[8]
};
```

预测一步状态：

```cpp
double x1[9];
```

参数注入 `p` 的布局也要同步更新。

### 6.5 `updateLastStateFromCommand()`

当前一阶更新逻辑类似：

```cpp
v_next = v + alpha * (v_cmd - v);
```

改成二阶状态更新。第一版可以使用 semi-implicit Euler：

```cpp
const double tau_v = std::max(0.05, params_.vel_lag_tau);
const double zeta_v = std::max(0.05, params_.vel_lag_zeta);

const double a_v_dot =
    (v_cmd - last_state_[3] - 2.0 * zeta_v * tau_v * last_state_[5]) /
    (tau_v * tau_v);

const double a_v_next = last_state_[5] + a_v_dot * dt;
const double v_next = last_state_[3] + a_v_next * dt;

last_state_[3] = v_next;
last_state_[5] = a_v_next;
last_state_[7] = v_cmd;
```

角速度通道同理：

```cpp
const double alpha_w_dot =
    (omega_cmd - last_state_[4] - 2.0 * zeta_w * tau_w * last_state_[6]) /
    (tau_w * tau_w);

const double alpha_w_next = last_state_[6] + alpha_w_dot * dt;
const double omega_next = last_state_[4] + alpha_w_next * dt;

last_state_[4] = omega_next;
last_state_[6] = alpha_w_next;
last_state_[8] = omega_cmd;
```

注意：如果有可靠的底盘速度反馈，`v/omega` 仍然应优先用实测反馈融合。`a_v/alpha_w` 可以先由模型递推估计，不必强行从差分速度计算，否则噪声会比较大。

### 6.6 `nav_params.yaml`

当前文件：

```text
src/my_navigation/nav_bringup/config/nav_params.yaml
```

已有：

```yaml
vel_lag_tau: 0.60
omega_lag_tau: 0.10
```

建议新增：

```yaml
vel_lag_zeta: 1.0
omega_lag_zeta: 1.0
```

第一版先用临界阻尼。确认模型稳定后，再根据实测过冲逐步调小。

## 7. 参数初值建议

建议第一版不要太激进：

```text
vel_lag_zeta: 1.0
omega_lag_zeta: 1.0
```

`tau` 的初值可以先沿用当前值，但要注意：二阶临界阻尼在相同 `tau` 下可能比一阶更慢。若发现预测响应明显慢于实车，可以逐步减小 `tau`。

建议调参顺序：

1. 固定 `zeta=1.0`，先调 `tau`，让预测上升速度接近实车。
2. 如果实车有明显过冲，而预测没有过冲，再把 `zeta` 从 `1.0` 往下调，例如 `0.9`、`0.8`、`0.7`。
3. 如果预测过冲太大或闭环摆动，增大 `zeta` 或增大 `tau`。
4. 模型对齐后，再调 `R_linear/R_angular` 和加速度约束，不要混在一起调。

## 8. 如何判断改动是否有效

不要只看路径跟踪主观效果，至少比较三条信号：

```text
v_cmd          NMPC 下发速度命令
v_meas         底盘实际速度反馈
v_pred_1step   NMPC 模型预测的一步后速度
```

有效的迹象：

```text
v_pred_1step 和 v_meas 的趋势更接近
过冲幅度预测更接近实车
回摆相位误差减小
路径跟踪中速度命令不再反复猛推/猛刹
```

无效或需要继续排查的迹象：

```text
v_pred 仍然单调，但 v_meas 明显过冲：zeta 可能太大，模型仍过阻尼
v_pred 过冲比 v_meas 大很多：zeta 太小或 tau 太小
所有曲线整体错开固定时间：可能存在纯延迟，不是二阶滞后能完全解决
速度反馈噪声很大导致 a_v 推断乱跳：不要直接差分速度作为 a_v
```

## 9. 修改后的测试流程

测试目标不是马上追求更快，而是先确认：

```text
NMPC 内部预测速度 v_pred 是否比一阶模型更接近真实速度 v_meas
```

如果预测速度不准，即使短时间路径效果看起来变好，也不建议继续调激进参数。

### 9.1 测试前参数建议

第一轮建议保守配置：

```yaml
vel_lag_zeta: 1.0
omega_lag_zeta: 1.0
odom_feedback_alpha: 0.2
```

说明：

```text
zeta=1.0          先用临界阻尼，避免模型自己引入过强振荡
alpha=0.2         小比例使用实测速度，避免反馈延迟/噪声过度影响 NMPC 初始状态
```

不要一开始使用：

```yaml
odom_feedback_alpha: 0.5
```

原因是：如果速度反馈本身有延迟或噪声，`0.5` 可能会让 NMPC 初始速度状态被滞后的测量值拖动，导致求解输出变差。

### 9.2 第一阶段：原地或直线速度阶跃测试

先不要直接跑复杂路径。建议在安全场地做单独速度响应测试：

```text
线速度：0 -> 0.3 -> 0 -> 0.6 -> 0
角速度：0 -> 0.8 -> 0 -> 1.5 -> 0
```

每个速度保持 3 到 5 秒，重复至少 3 次。

需要记录的信号：

```text
cmd_vel.linear.x
cmd_vel.angular.z
ChassisOdom.twist.twist.linear.x
ChassisOdom.twist.twist.angular.z
nmpc/speed_observation
```

其中 `nmpc/speed_observation` 里当前已有这些关键字段：

```text
twist.linear.x   = NMPC 下发线速度 cmd_vel.linear.x
twist.linear.y   = 底盘实测线速度 v_meas
twist.linear.z   = NMPC stage-1 预测线速度 v_pred_1step
twist.angular.x  = NMPC 输出线加速度 a_cmd
twist.angular.y  = 当前 tau_v
twist.angular.z  = stage-1 预测命令速度 v_cmd_state_pred_1step
```

如果后续要重点看角速度，建议再扩展一个角速度观测话题，或者复用额外字段记录：

```text
omega_cmd
omega_meas
omega_pred_1step
alpha_cmd
```

### 9.3 第二阶段：先扫 `odom_feedback_alpha`

固定二阶模型参数不变：

```yaml
vel_lag_tau: 当前值
omega_lag_tau: 当前值
vel_lag_zeta: 1.0
omega_lag_zeta: 1.0
```

只改变：

```yaml
odom_feedback_alpha: 0.0
odom_feedback_alpha: 0.2
odom_feedback_alpha: 0.3
odom_feedback_alpha: 0.4
```

不建议第一轮超过 `0.4`。如果 `/ChassisOdom` 确认是底盘内部速度、延迟很小、噪声也小，再考虑测试 `0.5` 或更高。

判断方法：

```text
alpha 太小：v_pred 和 v_meas 长时间分离，NMPC 对真实过冲不敏感
alpha 合适：v_pred 能被实测速度修正，但 cmd_vel 不明显抖动
alpha 太大：cmd_vel 反复修正、速度指令变抖、路径跟踪有滞后感
```

建议先选出一个稳定的 `odom_feedback_alpha`，之后调 `tau/zeta` 时保持它不变。

### 9.4 第三阶段：调二阶 `tau`

固定：

```yaml
odom_feedback_alpha: 第二阶段选出的值
vel_lag_zeta: 1.0
omega_lag_zeta: 1.0
```

再调：

```yaml
vel_lag_tau
omega_lag_tau
```

判断：

```text
v_pred 上升太慢：tau 可能太大
v_pred 上升太快：tau 可能太小
v_pred 相位明显晚于 v_meas：tau 可能偏大，或存在纯延迟/反馈延迟
```

每次只改一个通道。先调线速度 `vel_lag_tau`，再调角速度 `omega_lag_tau`。

### 9.5 第四阶段：再调二阶 `zeta`

只有当 `tau` 让上升速度基本对齐后，才开始调 `zeta`。

如果实车有过冲，但模型预测没有过冲：

```yaml
vel_lag_zeta: 0.9
vel_lag_zeta: 0.8
vel_lag_zeta: 0.7
```

逐步往下试，不要一次降太多。

判断：

```text
zeta 太大：预测曲线太稳，过冲/回摆不明显
zeta 合适：预测过冲幅度和回摆趋势接近实车
zeta 太小：预测过冲过大，NMPC 可能提前保守或反向修正过强
```

角速度通道同理调 `omega_lag_zeta`。

### 9.6 第五阶段：路径跟踪 A/B 测试

速度模型对齐后，再跑路径。

建议至少对比两组：

```text
A 组：当前一阶滞后模型或旧参数
B 组：二阶滞后模型新参数
```

测试要求：

```text
同一路径
同一速度档
同一负载
同一起点姿态
每组至少重复 3 次
```

观察指标：

```text
横向误差是否下降
转弯处是否减少冲出/回摆
cmd_vel 是否更平滑
速度过冲是否减少
到点时间是否没有明显变差
求解失败次数是否没有增加
```

### 9.7 推荐记录命令

示例：

```bash
ros2 bag record \
  /cmd_vel \
  /ChassisOdom \
  /nmpc/speed_observation \
  /nmpc/predicted_path
```

如果还有全局路径、局部路径或当前位姿话题，也建议一起记录，方便复盘路径误差。

### 9.8 停止测试的条件

出现以下情况应停止继续加大参数：

```text
cmd_vel 明显高频抖动
实际速度过冲比旧模型更严重
NMPC 求解失败次数增加
机器人在转弯处出现明显摆动
v_pred 与 v_meas 比一阶模型更不一致
```

此时优先回退：

```text
先降低 odom_feedback_alpha
再把 zeta 调回 1.0
最后回退 tau 或切回一阶模型
```

## 10. 重新生成和编译

修改 `model.py` 和 `export_ocp.py` 后，必须重新生成 acados solver。

参考流程：

```bash
cd /home/nuc/navigation2026
source .venv/bin/activate
export ACADOS_SOURCE_DIR=/home/nuc/dependency/acados
export LD_LIBRARY_PATH=/home/nuc/dependency/acados/lib:$LD_LIBRARY_PATH

cd /home/nuc/navigation2026/src/my_navigation/nav_components/model_ocp
python export_ocp.py
```

生成后检查：

```text
WHEELLEG_NMPC_NX = 9
WHEELLEG_NMPC_NU = 2
WHEELLEG_NMPC_NP = 26
```

然后重新编译：

```bash
cd /home/nuc/navigation2026
colcon build --packages-select nav_components --symlink-install
source install/setup.bash
```

## 11. 推荐实施顺序

1. 先新增二阶模型代码和参数，但保持 `zeta=1.0`。
2. 重新生成 solver，确认维度为 `NX=9, NP=26`。
3. 设置 `odom_feedback_alpha=0.2`，先做低速阶跃测试。
4. 固定二阶 `tau/zeta`，小范围扫描 `odom_feedback_alpha`。
5. 固定 `odom_feedback_alpha`，先调 `tau`，再调 `zeta`。
6. 对比 `v_pred_1step` 和 `v_meas`，确认模型匹配。
7. 模型匹配后，再做路径跟踪 A/B 测试。

## 12. 回滚策略

建议保留一阶模型分支或通过环境变量切换模型。

可选策略：

```text
NMPC_LAG_MODEL=first_order
NMPC_LAG_MODEL=second_order
```

这样如果二阶模型参数还没调好，可以快速回到当前一阶模型，不影响比赛或现场测试。

## 13. 最关键的设计提醒

二阶滞后不是为了让 NMPC 更复杂，而是为了让 NMPC 的预测更像真实底盘。

当前实车已经出现过冲和回摆，这说明“只用一阶慢慢追命令”的模型不够表达真实响应。二阶模型新增的 `a_v/alpha_w` 让 NMPC 能看到速度变化趋势；新增的 `zeta_v/zeta_w` 让 NMPC 能表达过冲和回摆程度。

因此，这次修改的目标不是单纯提高速度，而是提高：

```text
预测速度和实际速度的一致性
控制命令的可执行性
过冲/回摆场景下的闭环稳定性
```
