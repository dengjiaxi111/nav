# A* 贴拐角问题诊断（仅分析，不改代码）

日期：2026-04-07  
范围：`src/my_navigation/nav_components`（A* 与 costmap）

## 结论摘要

该问题不是单一原因，更像是「参数放宽 + 算法细节 + 栅格离散特性」叠加：

1. **参数配置原因（主因）**  
   当前 `planner.obstacle_threshold=100`，会把代价值 `99` 的内切危险区视为“可通行”，A* 允许贴障前进。
2. **代码逻辑原因（次因）**  
   A* 采用 8 邻域，但没有通用“对角穿角”拦截（只检查目标格是否可通行），会出现从障碍拐角斜切通过的倾向。
3. **场景/模型必然性（客观因素）**  
   栅格中心连线 + 最短路目标，本来就可能把路径压在可行域边界附近，尤其在狭窄转角或分辨率较粗时更明显。

用户补充澄清：

- 当前观察到的问题，主要出现在**原始 A\* 搜索结果**上。
- **B-spline 优化后的曲线外观整体还不错**，因此这次问题的主要矛头应指向 raw A\* 生成阶段，而不是先怀疑平滑器。

---

## 证据（代码定位）

### 1) `99` 被当成可通行（配置触发）

- 文件：`src/my_navigation/nav_bringup/config/nav_params.yaml`
  - `planner.obstacle_threshold: 100`
- 文件：`src/my_navigation/nav_components/src/simple_planner.cpp`
  - `isValid()`：`val >= 0 && val < obstacle_threshold_`
  - 当阈值=100 时，`val=99` 满足可通行。

### 2) 膨胀层把内切区编码为 99

- 文件：`src/my_navigation/nav_components/include/nav_components/costmap_inflater.hpp`
  - `computeCost()`：
    - `distance <= 0` -> `100`
    - `distance <= inscribed_radius` -> `99`
    - 之后衰减到 `1..98`

这意味着：若规划阈值设为 100，机器人可在“内切危险区(99)”行走。

### 3) 8 邻域未做通用 anti-corner-cut

- 文件：`src/my_navigation/nav_components/src/simple_planner.cpp`
  - `runAstar()` 使用 8 邻域 `dx/dy`。
  - 对对角步进，仅检查：
    - `map_manager_->isTransitionAllowed(...)`（当前主要用于 stair 单向过渡）
    - `isValid(nx, ny)`（目标格可通行）
  - **未看到**常规规则：对角移动时，还要要求 `(x+dx,y)` 与 `(x,y+dy)` 不可被障碍阻断。

### 4) `isTransitionAllowed()` 非通用避角逻辑

- 文件：`src/my_navigation/nav_components/src/layered_map_manager.cpp`
  - `isTransitionAllowed()` 仅在 `stair_layer.enable && enable_oneway_stair_down` 且有禁行边集合时生效；
  - 默认返回 `true`，并非全局对角防穿角机制。

---

## 分类判断（回答“代码/参数/必然”）

- **参数配置错误/不匹配：是（高置信）**
  - `obstacle_threshold=100` 与“希望远离膨胀区”的目标冲突。
- **代码编写缺陷：是（中高置信）**
  - A* 缺少通用 anti-corner-cut 检查。
- **特殊场景必然：部分是（中置信）**
  - 即使修正上面两点，离散栅格最短路仍可能在边界附近；但“贴得离谱”的主要问题通常可明显缓解。

---

## 不改代码的快速验证步骤

1. 只改参数（首选）：
   - `planner.obstacle_threshold: 99`
   - `planner.obstacle_check_threshold: 99`
2. 保持同一组起终点，观察 `/plan_astar_raw` 相对 `/costmap` 的最小间距。
3. 若仍明显对角“擦角”，则基本可确认是 A* 对角防穿角缺失导致的次因。

---

## 后续改进建议（暂不实施）

1. 参数层：
   - 将默认 `planner.obstacle_threshold` 收紧到 `<100`（建议 99 或更低）。
2. 算法层：
   - 在 `runAstar()` 加入对角步 anti-corner-cut 规则。
3. 成本层：
   - 视比赛场地再调 `cost_weight`，避免“为省几步贴障”。

---

## 要做的事情清单

### P0：先修正 raw A* 的硬错误

1. 调整规划阈值
   - 将 `planner.obstacle_threshold` 从 `100` 收紧到 `99`。
   - 目的：让 `99` 代价值内切危险区不可通行，避免 raw A* 贴着障碍角走。

2. 对齐路径检查阈值
   - 将 `planner.obstacle_check_threshold` 同步调整到 `99`。
   - 目的：避免“规划阶段可走，但验证阶段判障”或反过来的阈值不一致。

3. 增加 A* 对角防穿角
   - 在 `runAstar()` 中，对角扩展时同时检查两个正交邻格。
   - 若任一正交邻格不可通行，则禁止该对角扩展。
   - 目的：消除 raw A* 在障碍拐角的斜切穿越。

### P1：修复后复测 raw A*

4. 固定同一组起终点，重新观察 `/plan_astar_raw`
   - 重点看拐角附近是否仍明显压在障碍物边界。
   - 不先看 B-spline 曲线，优先看原始 A* 结果。

5. 对照 `/costmap` 进行视觉核查
   - 确认 raw A* 是否仍进入 `99` 区域边缘。
   - 确认对角穿角是否消失。

### P2：若仍有残余贴边，再做软优化

6. 评估 `planner.cost_weight`
   - 若 raw A* 仍偏向靠边但已不穿角，可再提高代价权重。
   - 这属于软优化，不替代 P0。

7. 评估膨胀参数是否需要微调
   - 仅在确认不是阈值与 anti-corner-cut 问题后再讨论。
   - 避免把算法缺陷误判为“膨胀不够大”。

### P3：结构治理

8. 统一障碍阈值治理
   - 检查并梳理 `LayeredMapManager` 与 `SimplePlanner` 的障碍阈值语义是否一致。
   - 目标：避免 costmap/ESDF/query/planner 各自使用不同阈值造成长期维护风险。



finished