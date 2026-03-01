# LayeredMapManager

`LayeredMapManager` 是导航侧统一地图入口，负责把静态地图、动态障碍和可选台阶语义融合成规划可用地图。

## 核心输出
- `static_map`：静态层原图。
- `fused_map`：静态层 + 动态层融合后的未膨胀占据图。
- `costmap`：由 `fused_map` 膨胀得到的代价地图。
- `esdf_vis`：可选 ESDF 可视化。

## 初始化流程
1. 加载静态地图（`map_file`）或创建空白静态图。
2. 初始化 `fused_map = static_map`。
3. 若启用 `stair_layer`，从 `stair_mask.yaml/pgm` 预计算清除索引。
4. 对 `fused_map` 执行台阶清除。
5. 构建 `costmap`（可选构建 ESDF）。

## 运行时更新流程
1. 接收动态层（通常来自 `/rog_map/map_2d`）。
2. 使用 `odom -> map` TF 将动态层投影到全局。
3. 重建 `fused_map` 并融合动态障碍。
4. 应用台阶清除（仅作用于 `fused_map`）。
5. 基于最新 `fused_map` 重建 `costmap`（与可选 ESDF）。

## stair_layer 语义
- 白色：忽略。
- 黑线：高侧。
- 灰线：低侧。
- 法向定义：`black - gray`。
- 清除距离：`stair_clear_perp_dist_m`（沿法向在 `fused_map` 清障）。

## 设计原则
- 地图语义处理集中在一个模块。
- 规划器和控制器只依赖统一 `MapInterface`。
- 参数驱动，可按赛场快速切换策略。
