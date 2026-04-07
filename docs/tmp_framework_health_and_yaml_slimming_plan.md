# 框架体检与 YAML 精简方案（临时文档）

> 目标：判断当前框架是否“设计错误”还是“功能叠加导致复杂化”，并给出参数精简清单。
> 当前状态：`Draft v0`（仅分析，不改代码）

## 1. 结论（先给判断）

1. 框架方向本身没有根本性错误：
- `nav_core` 抽象接口（planner/controller/recovery/map）是正确分层方向。
- `LayeredMapManager` 把静态/动态/台阶语义集中管理，这个架构是合理的。

2. 当前“臃肿”主要来自功能持续叠加后未做结构回收：
- 核心表现是 `nav_server.cpp` 单文件过大（1985 行），承担了太多职责。
- 参数集中在单个 `nav_params.yaml`（273 行），同时承载运行模式、调试、实验参数，导致可维护性下降。

结论：这是“结构债务 + 参数治理缺失”，不是路线错误。

## 2. 证据（客观指标）

1. 代码体量集中：
- `nav_server.cpp`: 1985 行
- `nmpc.cpp`: 1163 行
- `simple_planner.cpp`: 679 行

2. 单节点职责过载（`nav_server` 同时负责）：
- 目标接入（Action + RViz + 决策桥接）
- 地图加载/发布/动态层订阅
- 规划/控制/恢复状态机
- 台阶检测与固定速度策略
- 脱困策略
- 大量参数校验与日志

3. 参数配置混杂：
- 业务运行参数、调试参数、模式切换参数、实验参数在同一 YAML。

## 3. 框架主要问题（从工程视角）

1. 高耦合：
- 台阶策略、脱困策略、导航主循环都耦合在 `NavServer`。

2. 可替换性不足：
- 虽有 `ControllerBase` 抽象，但 `NavServer` 直接持有 `NMPC` 实例，控制器切换不是配置化。

3. 参数治理不足：
- 参数“声明即长期保留”，缺少生命周期（实验参数未下线、兼容参数未回收）。

4. 运行模式与节点参数混写：
- 某些仅用于 `run.launch.py` 的模式开关也写入 `nav_server` 参数文件，职责边界不清。

## 4. 结构优化方案（不改行为的前提下）

## 阶段A：先拆职责，不改算法

1. 从 `nav_server` 抽离模块：
- `GoalIngress`（Action/RViz/决策桥接）
- `MapRuntime`（地图装载、动态层、地图发布）
- `ControlPipeline`（planning->terrain->controller）
- `RecoveryPipeline`
- `EscapeHandler`

2. 把台阶逻辑迁移到 `TerrainController`（与你前文方案一致）。

## 阶段B：参数治理

1. 参数按文件拆分：
- `nav_core.yaml`（核心运行）
- `planner.yaml`
- `nmpc.yaml`
- `terrain_stair.yaml`
- `recovery_escape.yaml`
- `debug.yaml`（默认不加载）
- `mode_two_point_patrol.yaml`（仅巡逻模式加载）

2. 定义参数生命周期：
- `stable` / `experimental` / `deprecated`

## 阶段C：插件化与可替换性

1. `NavServer` 仅依赖接口，不直接绑定 `NMPC` 具体类。
2. 控制器由参数选择（至少支持 `nmpc` / `pure_pursuit` 两档）。

## 5. YAML 参数精简审计（重点）

以下按“可立即弃用 / 建议迁移 / 保留”分类。

## 5.1 可立即弃用（当前运行路径下基本无效）

1. `controller.lookahead_dist`
2. `controller.max_linear_vel`
3. `controller.max_angular_vel`

原因：
- 这些仅被 `PurePursuit` 声明读取；
- 当前 `nav_server` 固定使用 `NMPC`，未实例化 `PurePursuit`。

4. `planner.stair_align_expand_points`（在当前配置下）

原因：
- 代码逻辑为“仅当四个弧长窗口参数都<=0时才回退使用 expand_points”；
- 当前 YAML 四个窗口参数均 > 0，`expand_points` 处于实际失效状态。

5. `yaw_tolerance`（当前 NMPC 控制链路下）

原因：
- 参数会传入 `controller_.setTolerance(xy, yaw)`；
- 但当前实际控制器 `NMPC` 的到达判定只用 `xy_tolerance`，未使用 `yaw_tolerance` 参与成功判定。

## 5.2 建议迁移（不是无效，但不应放在当前总 YAML）

1. `use_static_map_odom`
2. `debug_reset_odom_to_base_link`

原因：
- 这两个本质是启动/模式参数（被 `run.launch.py` 用于选择启动节点），不属于 `nav_server` 运行参数。

3. `two_point_patrol.*` 整段

原因：
- 仅 `two_point_patrol` 节点使用，建议放到独立 `two_point_patrol.yaml`。

4. `path_io.*` 整段

原因：
- 属于调试/离线复用能力，建议移到 `debug.yaml` 或 `planner_experimental.yaml`。

5. 地图/台阶调试输出参数：
- `enable_performance_logging`
- `special_terrain.publish_stair_debug_markers`
- `special_terrain.debug_marker_max_segments`
- `nmpc.publish_predicted_path`

原因：
- 调试专用，建议默认关闭并迁移到 `debug.yaml`。

6. `map_file`（在标准 `run.launch.py` 路径下）

原因：
- `navigation.launch.py` 会用 launch 参数再次覆盖 `map_file`；
- `run.launch.py` 仅传 `params_file/use_sim_time`，因此 `map_file` 实际采用 navigation.launch 的默认值。

## 5.3 建议保留（核心能力参数）

1. 核心导航：`control_rate/goal_timeout/goal_tolerance/controller_timeout/...`
2. 地图：`enable_static_layer/enable_dynamic_layer/dynamic_layer_topic/inflation.*`
3. 规划：`planner.*`（除已判定可弃用项）
4. NMPC：`nmpc.*` 主体
5. 恢复与脱困：`recovery.*`、`escape.*`
6. 台阶语义与策略：`special_terrain.*` 主体

## 6. 高优先级“精简第一刀”（最小风险）

建议先做这 6 件事：
1. 删除 `controller.*` 三个参数（或标记 deprecated 并不再展示）。
2. 删除/废弃 `planner.stair_align_expand_points`（若确认全面使用弧长窗口）。
3. 把 `two_point_patrol.*` 移到独立 YAML。
4. 把 `use_static_map_odom/debug_reset_odom_to_base_link` 从 nav 参数文件移到 launch 参数或 `run_mode.yaml`。
5. 将 debug 参数集中到 `debug.yaml` 并默认不加载。
6. 增加 `PARAMETERS.md`，记录每个参数归属模块与默认 profile。

## 7. 风险与控制

1. 风险：误删低频但关键参数。
- 控制：先标记 deprecated，一版后再物理删除。

2. 风险：多 YAML 导致部署复杂。
- 控制：提供 `minimal` / `competition` / `debug` 三个 profile launch 入口。

3. 风险：旧脚本依赖原参数路径。
- 控制：保留参数别名兼容层并给出迁移提示。

---

本文件为分析与治理方案，不包含代码修改。
