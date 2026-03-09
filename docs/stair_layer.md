# stair_layer

## 作用

- 基于手工 `stair_mask.pgm`，在导航融合地图中强制清除台阶跨越带的代价。
- 目标：让 2D 路径搜索可生成跨台阶路径。

## mask 定义

- 白色(255)：忽略。
- 双向台阶黑线(高侧)：像素值在 `[black_min, black_max]`。
- 双向台阶灰线(低侧)：像素值在 `[gray_min, gray_max]`。
- 单向台阶黑线(高侧)：像素值在 `[oneway_black_min, oneway_black_max]`。
- 单向台阶灰线(低侧)：像素值在 `[oneway_gray_min, oneway_gray_max]`。
- 法向定义：`n = normalize(black - gray)`（从低侧指向高侧）。

## 清除规则

- 参数 `stair_clear_perp_dist_m` 表示沿法向强制清除距离（米）。
- 对每个黑线像素，搜索附近灰线像素得到方向；沿法向生成清除带。
- 清除仅作用于 `fused_map`（未膨胀融合图）。
- `costmap` 由最新 `fused_map` 重建得到，不再单独做二次台阶清除。

## 单向规则

- 打开 `enable_oneway_stair_down` 后：
  - 单向台阶允许 `高 -> 低`。
  - 单向台阶禁止 `低 -> 高`。
- 该约束在 A* 邻接扩展时生效（有向边约束），不是后处理裁剪。
- 调试可视化话题：`/stair_layer/forbidden_transitions`（`MarkerArray`）。
  - 红色线段：被禁止的搜索方向（`low -> high`）。
  - 黄色点：线段终点（被禁止方向的 `to` 端）。

## 配置

在 `nav_params.yaml` 中配置：

- `special_terrain.enable_stair_layer`
- `special_terrain.stair_mask_yaml`
- `special_terrain.stair_clear_perp_dist_m`
- `special_terrain.black_min / black_max`
- `special_terrain.gray_min / gray_max`
- `special_terrain.pair_search_radius_cells`
- `special_terrain.enable_oneway_stair_down`
- `special_terrain.oneway_black_min / oneway_black_max`
- `special_terrain.oneway_gray_min / oneway_gray_max`
- `special_terrain.publish_stair_debug_markers`
- `special_terrain.debug_marker_max_segments`

## 使用步骤

1. 准备 `stair_mask.pgm` + 对应 `stair_mask.yaml`（含 `image/resolution/origin`）。
2. 在 `nav_params.yaml` 打开 `enable_stair_layer` 并设置 `stair_mask_yaml`。
3. 启动导航，优先检查 `/fused_map` 是否出现跨台阶通行带，再确认 `/costmap` 的最终可通行性。
