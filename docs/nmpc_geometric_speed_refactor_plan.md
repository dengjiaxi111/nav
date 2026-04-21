# NMPC 几何速度曲线生成重构设计文档

**文档状态**: 草案 (Draft) / 规划中
**日期**: 2026-04-21
**目标模块**: `nav_components` -> `NMPC` 

## 1. 背景与问题陈述 (Background & Problem)

当前 NMPC 控制器的参考速度生成和整形逻辑存在过度约束（Over-constrained）的问题。最初为了保证实车运行安全，在将期望轨迹送入 NMPC 求解器之前，叠加了大量基于规则（Rule-based）的状态反馈降速逻辑。这导致了以下问题：

1. **逻辑越俎代庖（与 NMPC 理念冲突）**：
   NMPC 本质是一个带约束的优化求解器，本身就能通过设置状态权重（$Q_{pos}, Q_{ori}$）和动力学硬约束（最大加速度、最大速度）来自动降低速度以消除轨迹误差。但当前外层又增加了很多基于航向误差、横向误差的缩减，造成“双重限制”，导致机器人频繁出现异常卡顿和过于保守的减速。
   
2. **曲率限速重叠**：
   代码中同时存在基于 `curvature_decay_kappa_ref` 经验倍率衰减，以及基于 `speed_profile_max_lateral_accel` 横向加速度限制的运动学衰减，两套逻辑相互干扰。

3. **近终点逻辑过于臃肿**：
   依靠多个布尔型状态标志（如 `goal_brake_latched_`、`enable_goal_speed_guard` 等）和多个缩放系数强行框死终端速度。状态机式的修剪容易引起死锁或异常跳变。

---

## 2. 重构核心思想 (Refactoring Philosophy)

**核心指导原则：“将几何空间与动态状态解耦”**

在传入 NMPC 之前，参考速度生成器（Speed Profiler）**只关心路径的几何形态**，不关心机器人当前的状态偏差（航向偏差、横向偏差）。
状态偏差的收敛完全交给 NMPC 优化器利用权重和 $cost\ function$ 自动求解。

**期望的输入链路**:
`[全局平滑路径]` -> `[运动学几何限速算法]` -> `[终点平滑刹车生成]` -> `[提取当前 NMPC Horizon]` -> `[送入求解器]`

---

## 3. 具体重构方案 (Implementation Plan)

### 阶段一：清理基于状态偏差的降速补丁（做减法）
**目标**：把状态机纠偏的责任还给 NMPC 求解器。
*   **废除项**：
    *   移除 `lateral_error_threshold` 及横向误差带来的 $v_{ref}$ 缩减。
    *   移除 `heading_slowdown_start`、`heading_slowdown_min_factor` 及相关的航向误差降速逻辑。
*   **保留项**：
    *   保留 `pivot_turn_heading_thresh` (原地转向标志)。当航向误差过极大、无法仅靠前馈加减速纠正时，依然需要顶层强行下发 $v_{ref} = 0$, $\omega_{ref} = steer$ 实现差速转向对齐。

### 阶段二：合并弯道曲率限速机制（物理模型化）
**目标**：用一个严格的动力学公式取代所有的经验倍率相乘。
*   **废除项**：
    *   移除旧有的 `enable_curvature_speed_decay` 和 `curvature_decay_kappa_ref` 系统。
*   **统一逻辑 (`speed_profile`)**:
    *   提取前方局部路径的曲率 $\kappa$。
    *   利用物理约束 `speed_profile_max_lateral_accel` ($a_{y\_max}$) 计算理论最大速度：
        $$ v_{limit\_curve} = \sqrt{\frac{a_{y\_max}}{\max(|\kappa|, 1e-4)}} $$
    *   基础参考速度：
        $$ v_{ref} = \min(v_{cruise}, v_{limit\_curve}, v_{limit\_max}) $$
    *   通过保底速度 `speed_profile_v_min` 避免机器人死锁。

### 阶段三：简化终点平滑减速 (Smooth Goal Deceleration)
**目标**：用平滑的运动学减速曲线（开方曲线）代替繁琐的安全包络。
*   **废除项**：
    *   移除 `enable_goal_speed_guard` 及相关复杂的倍率逻辑 (`goal_speed_guard_dist_scale`, `_decel_scale`, `_abs_floor`)。
    *   移除减速锁存变量 `goal_brake_latched_`（只要定位和里程计健康，不需要在控制器层强行锁死速度上限）。
*   **新逻辑**：
    从距离终点 $S_{brake}$（例如基于最大恒定减速度 $a_{d}$ 反算得到的制动距离）处开始，采用以下公式直接压低参考速度列：
    $$ v_{ref\_goal} = \sqrt{2 \cdot a_{d} \cdot \Delta s_{to\_goal}} $$
    其中 $\Delta s_{to\_goal}$ 是该控制点沿路径剩余的弧长。这会生成一条完美的平滑刹车参考线。

---

## 4. 参数清理清单 (Parameters to Deprecate)

在 `nav_params.yaml` 中，重构后有望彻底删除或屏蔽以下参数：

```yaml
# 拟删除的冗余降速补丁
- nmpc.enable_heading_slowdown
- nmpc.heading_slowdown_start
- nmpc.heading_slowdown_min_factor
- nmpc.lateral_error_threshold

# 拟删除的旧版曲率衰减
- nmpc.enable_curvature_speed_decay
- nmpc.curvature_decay_kappa_ref
- nmpc.curvature_decay_min_factor

# 拟简化的终点包络参数
- nmpc.enable_goal_speed_guard
- nmpc.goal_speed_guard_dist_scale
- nmpc.goal_speed_guard_decel_scale
- nmpc.goal_speed_guard_abs_floor
```

## 5. 预期收益 (Expected Benefits)

1. **响应更流畅**：机器人偏离路径时，不再出现无故“急刹车”的顿挫感，NMPC 生成的 $a_{cmd}$ 会极其连贯。
2. **调参变简单**：将关注点集中在真正起作用的权重矩阵（$Q, R$）上，不用再在多个降速倍率之间寻找微妙的平衡。
3. **代码易维护**：`extractLocalReference` 阶段的代码行数将缩减一半，职责将变得单一（仅负责速度前置规划）。
