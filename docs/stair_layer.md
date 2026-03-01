# stair_layer

## 作用
- 基于手工 `stair_mask.pgm`，在导航融合地图中强制清除台阶跨越带的代价。
- 目标：让 2D 路径搜索可生成跨台阶路径。

## mask 定义
- 白色(255)：忽略。
- 黑线(高侧)：像素值在 `[black_min, black_max]`。
- 灰线(低侧)：像素值在 `[gray_min, gray_max]`。
- 法向定义：`n = normalize(black - gray)`（从低侧指向高侧）。

## 清除规则
- 参数 `stair_clear_perp_dist_m` 表示沿法向强制清除距离（米）。
- 对每个黑线像素，搜索附近灰线像素得到方向；沿法向生成清除带。
- 清除仅作用于 `fused_map`（未膨胀融合图）。
- `costmap` 由最新 `fused_map` 重建得到，不再单独做二次台阶清除。

## 配置
在 `nav_params.yaml` 中配置：
- `special_terrain.enable_stair_layer`
- `special_terrain.stair_mask_yaml`
- `special_terrain.stair_clear_perp_dist_m`
- `special_terrain.black_min / black_max`
- `special_terrain.gray_min / gray_max`
- `special_terrain.pair_search_radius_cells`

## 使用步骤
1. 准备 `stair_mask.pgm` + 对应 `stair_mask.yaml`（含 `image/resolution/origin`）。
2. 在 `nav_params.yaml` 打开 `enable_stair_layer` 并设置 `stair_mask_yaml`。
3. 启动导航，优先检查 `/fused_map` 是否出现跨台阶通行带，再确认 `/costmap` 的最终可通行性。
