// nav_components/src/layered_map_manager.cpp
// 分层地图管理器实现

#include "nav_components/layered_map_manager.hpp"
#include "nav_components/map_image_io.hpp"
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <unordered_set>

namespace nav_components {

LayeredMapManager::LayeredMapManager()
    : logger_(rclcpp::get_logger("layered_map_manager")) {}

void LayeredMapManager::initialize(rclcpp::Node* node,
                                    std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    node_ = node;
    tf_buffer_ = tf_buffer;
    logger_ = node_->get_logger();
    
    // 读取坐标系参数
    node_->get_parameter_or<std::string>("map_frame", map_frame_, map_frame_);
    node_->get_parameter_or<std::string>("odom_frame", odom_frame_, odom_frame_);

    // 读取性能日志开关（默认 false）
    node_->get_parameter_or<bool>("enable_performance_logging", enable_performance_logging_, enable_performance_logging_);
    
    RCLCPP_INFO(logger_, "LayeredMapManager initialized (map: %s, odom: %s, perf_log: %s)",
                map_frame_.c_str(), odom_frame_.c_str(), 
                enable_performance_logging_ ? "enabled" : "disabled");
}

void LayeredMapManager::setStairLayerConfig(const StairLayerConfig& cfg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    stair_layer_cfg_ = cfg;

    if (!stair_layer_cfg_.enable) {
        stair_clear_indices_.clear();
        stair_mask_loaded_ = false;
        return;
    }

    if (stair_layer_cfg_.clear_perp_dist_m <= 0.0) {
        RCLCPP_WARN(logger_, "stair_layer clear_perp_dist_m<=0, 自动修正为0.1m");
        stair_layer_cfg_.clear_perp_dist_m = 0.1;
    }

    rebuildStairLayerCache();
}

bool LayeredMapManager::loadStairMaskFromYaml(const std::string& yaml_path) {
    MapImageData mask_data;
    if (!MapImageIO::loadYamlAndPGM(yaml_path, mask_data, logger_)) {
        return false;
    }

    stair_mask_width_ = mask_data.width;
    stair_mask_height_ = mask_data.height;
    stair_mask_max_val_ = mask_data.max_val;
    stair_mask_resolution_ = mask_data.resolution;
    stair_mask_origin_x_ = mask_data.origin_x;
    stair_mask_origin_y_ = mask_data.origin_y;
    stair_mask_pixels_ = std::move(mask_data.pixels);
    stair_mask_loaded_ = true;
    loaded_stair_mask_yaml_ = yaml_path;

    RCLCPP_INFO(logger_, "stair mask 加载成功: %dx%d @ %.3f (%s)",
                stair_mask_width_, stair_mask_height_, stair_mask_resolution_,
                mask_data.image_path.c_str());
    return true;
}

bool LayeredMapManager::worldToGlobalIndex(double wx, double wy, int& idx) const {
    if (width_ <= 0 || height_ <= 0 || resolution_ <= 0.0) {
        return false;
    }
    int gx = static_cast<int>((wx - origin_x_) / resolution_);
    int gy = static_cast<int>((wy - origin_y_) / resolution_);
    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
        return false;
    }
    idx = gy * width_ + gx;
    return true;
}

void LayeredMapManager::rebuildStairLayerCache() {
    stair_clear_indices_.clear();

    if (!stair_layer_cfg_.enable || stair_layer_cfg_.mask_yaml_path.empty()) {
        return;
    }

    if (!static_map_) {
        return;
    }

    if (!stair_mask_loaded_ || loaded_stair_mask_yaml_ != stair_layer_cfg_.mask_yaml_path) {
        if (!loadStairMaskFromYaml(stair_layer_cfg_.mask_yaml_path)) {
            RCLCPP_WARN(logger_, "stair_layer 已启用但 mask 加载失败，跳过覆盖");
            return;
        }
    }

    if (stair_mask_width_ <= 0 || stair_mask_height_ <= 0 || stair_mask_pixels_.empty()) {
        return;
    }

    std::vector<uint8_t> class_map(static_cast<size_t>(stair_mask_width_ * stair_mask_height_), 0);
    std::vector<std::pair<int, int>> black_cells;
    black_cells.reserve(1024);

    for (int my = 0; my < stair_mask_height_; ++my) {
        for (int mx = 0; mx < stair_mask_width_; ++mx) {
            int pgm_idx = (stair_mask_height_ - 1 - my) * stair_mask_width_ + mx;
            int mask_idx = my * stair_mask_width_ + mx;
            int v = stair_mask_pixels_[pgm_idx];

            if (v >= stair_layer_cfg_.black_min && v <= stair_layer_cfg_.black_max) {
                class_map[mask_idx] = 1;
                black_cells.emplace_back(mx, my);
            } else if (v >= stair_layer_cfg_.gray_min && v <= stair_layer_cfg_.gray_max) {
                class_map[mask_idx] = 2;
            }
        }
    }

    if (black_cells.empty()) {
        RCLCPP_WARN(logger_, "stair_layer: mask 中未找到黑线像素");
        return;
    }

    std::unordered_set<int> clear_idx_set;
    clear_idx_set.reserve(black_cells.size() * 4);

    const int search_r = std::max(1, stair_layer_cfg_.pair_search_radius_cells);
    const int clear_steps = std::max(1, static_cast<int>(std::ceil(stair_layer_cfg_.clear_perp_dist_m / resolution_)));

    for (size_t i = 0; i < black_cells.size(); ++i) {
        int bx = black_cells[i].first;
        int by = black_cells[i].second;

        double best_dist2 = std::numeric_limits<double>::max();
        int gx_best = -1;
        int gy_best = -1;

        for (int dy = -search_r; dy <= search_r; ++dy) {
            for (int dx = -search_r; dx <= search_r; ++dx) {
                int gx = bx + dx;
                int gy = by + dy;
                if (gx < 0 || gx >= stair_mask_width_ || gy < 0 || gy >= stair_mask_height_) {
                    continue;
                }
                int idx = gy * stair_mask_width_ + gx;
                if (class_map[idx] != 2) {
                    continue;
                }
                double d2 = static_cast<double>(dx * dx + dy * dy);
                if (d2 < best_dist2) {
                    best_dist2 = d2;
                    gx_best = gx;
                    gy_best = gy;
                }
            }
        }

        if (gx_best < 0 || gy_best < 0) {
            continue;
        }

        double bwx = stair_mask_origin_x_ + (bx + 0.5) * stair_mask_resolution_;
        double bwy = stair_mask_origin_y_ + (by + 0.5) * stair_mask_resolution_;
        double gwx = stair_mask_origin_x_ + (gx_best + 0.5) * stair_mask_resolution_;
        double gwy = stair_mask_origin_y_ + (gy_best + 0.5) * stair_mask_resolution_;

        double nx = bwx - gwx;
        double ny = bwy - gwy;
        double norm = std::hypot(nx, ny);
        if (norm < 1e-5) {
            continue;
        }
        nx /= norm;
        ny /= norm;

        for (int s = 0; s <= clear_steps; ++s) {
            double wx = bwx + nx * (s * resolution_);
            double wy = bwy + ny * (s * resolution_);
            int global_idx = -1;
            if (worldToGlobalIndex(wx, wy, global_idx)) {
                clear_idx_set.insert(global_idx);
            }
        }

    }

    stair_clear_indices_.assign(clear_idx_set.begin(), clear_idx_set.end());

    RCLCPP_INFO(logger_, "stair_layer: 黑线=%zu, 清除栅格=%zu",
                black_cells.size(), stair_clear_indices_.size());
}

void LayeredMapManager::applyStairLayerPolicy() {
    if (!stair_layer_cfg_.enable || stair_clear_indices_.empty()) {
        return;
    }

    if (fused_map_) {
        for (int idx : stair_clear_indices_) {
            if (idx >= 0 && idx < static_cast<int>(fused_map_->data.size())) {
                fused_map_->data[idx] = 0;
            }
        }
    }
}

bool LayeredMapManager::loadStaticMap(const std::string& yaml_path,
                                       const InflationParams& params) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    static_map_ = StaticMapLoader::load(yaml_path, logger_);
    if (!static_map_) {
        RCLCPP_ERROR(logger_, "Failed to load static map from: %s", yaml_path.c_str());
        return false;
    }
    
    inflation_params_ = params;
    
    // 提取地图元数据
    resolution_ = static_map_->info.resolution;
    origin_x_ = static_map_->info.origin.position.x;
    origin_y_ = static_map_->info.origin.position.y;
    width_ = static_map_->info.width;
    height_ = static_map_->info.height;
    map_frame_ = static_map_->header.frame_id.empty() ? "map" : static_map_->header.frame_id;
    
    // 初始化融合地图（与静态地图相同大小）
    fused_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    *fused_map_ = *static_map_;  // 初始时等于静态地图

    rebuildStairLayerCache();
    applyStairLayerPolicy();
    
    // 构建costmap
    rebuildCostmap();
    
    // 如果启用ESDF，构建初始ESDF
    if (esdf_enabled_) {
        rebuildEsdf();
    }
    
    RCLCPP_INFO(logger_, "Static map loaded: %dx%d @ %.3f m/cell, inflation_radius=%.2f",
                width_, height_, resolution_, params.inflation_radius);
    return true;
}

void LayeredMapManager::setStaticMap(nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    if (!map) return;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    static_map_ = map;
    
    resolution_ = map->info.resolution;
    origin_x_ = map->info.origin.position.x;
    origin_y_ = map->info.origin.position.y;
    width_ = map->info.width;
    height_ = map->info.height;
    map_frame_ = map->header.frame_id.empty() ? "map" : map->header.frame_id;
    
    // 初始化融合地图
    fused_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    *fused_map_ = *static_map_;

    rebuildStairLayerCache();
    applyStairLayerPolicy();
    
    rebuildCostmap();
    if (esdf_enabled_) {
        rebuildEsdf();
    }
}

void LayeredMapManager::setInflationParams(const InflationParams& params) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    inflation_params_ = params;
    if (fused_map_) {
        rebuildCostmap();
    }
}

void LayeredMapManager::createBlankStaticMap(double width_m, double height_m,
                                              double resolution, const InflationParams& params) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 计算栅格尺寸
    int width = static_cast<int>(std::ceil(width_m / resolution));
    int height = static_cast<int>(std::ceil(height_m / resolution));
    
    // 创建空白地图
    static_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    static_map_->header.frame_id = map_frame_;
    static_map_->header.stamp = node_->get_clock()->now();
    
    static_map_->info.resolution = resolution;
    static_map_->info.width = width;
    static_map_->info.height = height;
    
    // 地图中心在原点
    static_map_->info.origin.position.x = -width_m / 2.0;
    static_map_->info.origin.position.y = -height_m / 2.0;
    static_map_->info.origin.position.z = 0.0;
    static_map_->info.origin.orientation.w = 1.0;
    
    // 全部初始化为自由空间 (0)
    static_map_->data.resize(width * height, 0);
    
    // 更新成员变量
    resolution_ = resolution;
    origin_x_ = static_map_->info.origin.position.x;
    origin_y_ = static_map_->info.origin.position.y;
    width_ = width;
    height_ = height;
    
    // 初始化融合地图
    fused_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    *fused_map_ = *static_map_;

    rebuildStairLayerCache();
    applyStairLayerPolicy();
    
    // 设置膨胀参数并构建Costmap
    inflation_params_ = params;
    rebuildCostmap();
    
    if (esdf_enabled_) {
        rebuildEsdf();
    }
    
    RCLCPP_INFO(logger_, "创建空白静态地图: %dx%d (%.1fm x %.1fm) @ %.3f m/cell",
                width, height, width_m, height_m, resolution);
}

void LayeredMapManager::updateDynamicLayer(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& local_map) {
    
    if (!local_map || !static_map_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 1. 查询 odom -> map 变换
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(
            map_frame_, odom_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000,
            "Cannot get %s->%s transform: %s", odom_frame_.c_str(), map_frame_.c_str(), ex.what());
        return;
    }
    
    // 2. 重置融合地图
    if (!fused_map_) {
        fused_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    }
    
    if (static_layer_enabled_) {
        // 静态层启用：融合地图初始化为静态地图
        *fused_map_ = *static_map_;
    } else {
        // 静态层禁用：融合地图初始化为空白（仅保留动态层）
        *fused_map_ = *static_map_;  // 复用结构
        std::fill(fused_map_->data.begin(), fused_map_->data.end(), 0);  // 清空为free space
    }
    fused_map_->header.stamp = node_->get_clock()->now();
    
    // 3. 变换参数
    double tx = transform.transform.translation.x;
    double ty = transform.transform.translation.y;
    double yaw = tf2::getYaw(transform.transform.rotation);
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    
    // 局部地图参数
    double local_res = local_map->info.resolution;
    double local_ox = local_map->info.origin.position.x;
    double local_oy = local_map->info.origin.position.y;
    int local_w = local_map->info.width;
    int local_h = local_map->info.height;
    
    // 4. 记录变化区域用于增量ESDF更新
    std::vector<int> changed_cells;
    changed_cells.reserve(local_w * local_h / 4);  // 预估25%变化
    
    DynamicLayerBounds new_bounds;
    new_bounds.min_x = width_;
    new_bounds.min_y = height_;
    new_bounds.max_x = 0;
    new_bounds.max_y = 0;
    
    // 5. 遍历局部地图，投影到全局地图
    for (int ly = 0; ly < local_h; ++ly) {
        for (int lx = 0; lx < local_w; ++lx) {
            int local_idx = ly * local_w + lx;
            int8_t local_val = local_map->data[local_idx];
            
            // 跳过unknown和free（只叠加障碍物）
            if (local_val <= 0) {
                continue;
            }
            
            // 局部栅格中心的odom坐标
            double odom_x = local_ox + (lx + 0.5) * local_res;
            double odom_y = local_oy + (ly + 0.5) * local_res;
            
            // 变换到map坐标系
            double map_x = cos_yaw * odom_x - sin_yaw * odom_y + tx;
            double map_y = sin_yaw * odom_x + cos_yaw * odom_y + ty;
            
            // 计算全局地图索引
            int gx = static_cast<int>((map_x - origin_x_) / resolution_);
            int gy = static_cast<int>((map_y - origin_y_) / resolution_);
            
            // 边界检查
            if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
                continue;
            }
            
            int global_idx = gy * width_ + gx;
            
            // 融合策略
            int8_t fused_val;
            if (static_layer_enabled_) {
                // 静态层启用：取max（保守，障碍物优先）
                int8_t static_val = static_map_->data[global_idx];
                fused_val = std::max(static_val, local_val);
            } else {
                // 静态层禁用：直接使用动态层的值
                fused_val = local_val;
            }
            
            // 记录变化
            if (fused_map_->data[global_idx] != fused_val) {
                fused_map_->data[global_idx] = fused_val;
                changed_cells.push_back(global_idx);
                
                // 更新边界
                new_bounds.min_x = std::min(new_bounds.min_x, gx);
                new_bounds.max_x = std::max(new_bounds.max_x, gx);
                new_bounds.min_y = std::min(new_bounds.min_y, gy);
                new_bounds.max_y = std::max(new_bounds.max_y, gy);
            }
        }
    }
    
    new_bounds.valid = !changed_cells.empty();
    
    // 6. 清除上一次动态层的残留（在新边界外的部分）
    // 保存旧边界用于后续计算
    DynamicLayerBounds old_bounds = last_dynamic_bounds_;
    
    if (old_bounds.valid) {
        for (int gy = old_bounds.min_y; gy <= old_bounds.max_y; ++gy) {
            for (int gx = old_bounds.min_x; gx <= old_bounds.max_x; ++gx) {
                // 跳过当前边界内的栅格（已经更新过了）
                if (new_bounds.valid &&
                    gx >= new_bounds.min_x && gx <= new_bounds.max_x &&
                    gy >= new_bounds.min_y && gy <= new_bounds.max_y) {
                    continue;
                }
                
                int idx = gy * width_ + gx;
                if (idx >= 0 && idx < width_ * height_) {
                    // 恢复基础值
                    int8_t base_val = static_layer_enabled_ ? static_map_->data[idx] : 0;
                    if (fused_map_->data[idx] != base_val) {
                        fused_map_->data[idx] = base_val;
                        changed_cells.push_back(idx);
                    }
                }
            }
        }
    }
    
    // 更新边界记录
    last_dynamic_bounds_ = new_bounds;

    applyStairLayerPolicy();
    
    // 7. 共享EDT计算优化：ESDF和Costmap复用同一次距离场计算
    auto t_start = std::chrono::high_resolution_clock::now();
    
    // 构建ESDF（含EDT计算）
    if (esdf_enabled_) {
        esdf_ = std::make_shared<EsdfMap>();
        esdf_->buildFromOccupancy(fused_map_, obstacle_threshold_);
    }
    auto t_esdf = std::chrono::high_resolution_clock::now();
    
    // 从ESDF复用距离场生成Costmap
    if (esdf_ && esdf_->hasDistance()) {
        rebuildCostmapFromEsdf();
    } else {
        rebuildCostmap();  // Fallback
    }
    auto t_costmap = std::chrono::high_resolution_clock::now();
    
    // 生成ESDF可视化
    if (esdf_enabled_ && esdf_) {
        esdf_vis_ = esdf_->toOccupancyGrid(esdf_vis_max_dist_);
        if (esdf_vis_) {
            esdf_vis_->header.frame_id = map_frame_;
        }
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    
    // 性能日志（根据开关决定是否输出）
    if (enable_performance_logging_) {
        auto dur_esdf = std::chrono::duration_cast<std::chrono::microseconds>(t_esdf - t_start);
        auto dur_costmap = std::chrono::duration_cast<std::chrono::microseconds>(t_costmap - t_esdf);
        auto dur_vis = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_costmap);
        auto dur_total = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
        
        RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 1000,
            "[共享EDT] ESDF: %.2f ms | Costmap: %.2f ms | Vis: %.2f ms | Total: %.2f ms (节省: %.0f%%)",
            dur_esdf.count() / 1000.0, dur_costmap.count() / 1000.0,
            dur_vis.count() / 1000.0, dur_total.count() / 1000.0,
            100.0 * (103.0 - dur_total.count() / 1000.0) / 103.0);
    }
}

void LayeredMapManager::rebuildFusedMap() {
    // 当前实现：在updateDynamicLayer中直接完成
    // 此函数保留用于未来扩展（如多动态层融合）
}

void LayeredMapManager::rebuildCostmap() {
    if (!fused_map_) return;
    costmap_ = CostmapInflater::inflate(fused_map_, inflation_params_, obstacle_threshold_);
    if (costmap_) {
        costmap_->header.frame_id = map_frame_;
    }
}

void LayeredMapManager::rebuildCostmapFromEsdf() {
    if (!fused_map_ || !esdf_) return;
    
    // 从ESDF获取距离场数据
    std::vector<float> distances(width_ * height_);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int idx = y * width_ + x;
            distances[idx] = esdf_->getDistanceFast(x, y);
        }
    }
    
    // 使用距离场直接生成costmap（无需重复EDT计算）
    costmap_ = CostmapInflater::fromEsdf(
        distances, width_, height_, resolution_,
        origin_x_, origin_y_, inflation_params_);
    
    if (costmap_) {
        costmap_->header.frame_id = map_frame_;
        costmap_->header.stamp = node_->get_clock()->now();
    }
}

void LayeredMapManager::rebuildEsdf() {
    // 仅用于初始化时的ESDF构建（loadStaticMap/setStaticMap调用）
    // 运行时的ESDF在updateDynamicLayer中与costmap共享EDT构建
    if (!fused_map_) return;
    
    esdf_ = std::make_shared<EsdfMap>();
    esdf_->buildFromOccupancy(fused_map_, obstacle_threshold_);
    esdf_vis_ = esdf_->toOccupancyGrid(esdf_vis_max_dist_);
    if (esdf_vis_) {
        esdf_vis_->header.frame_id = map_frame_;
    }
}

bool LayeredMapManager::transformOdomToMap(double ox, double oy,
                                            double& mx, double& my) const {
    if (!tf_buffer_) {
        return false;
    }
    
    try {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero);
        
        double tx = transform.transform.translation.x;
        double ty = transform.transform.translation.y;
        double yaw = tf2::getYaw(transform.transform.rotation);
        
        mx = std::cos(yaw) * ox - std::sin(yaw) * oy + tx;
        my = std::sin(yaw) * ox + std::cos(yaw) * oy + ty;
        return true;
    } catch (const tf2::TransformException&) {
        return false;
    }
}

// ============ 获取地图 ============

nav_msgs::msg::OccupancyGrid::SharedPtr LayeredMapManager::getStaticMap() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return static_map_;
}

nav_msgs::msg::OccupancyGrid::SharedPtr LayeredMapManager::getFusedMap() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return fused_map_;
}

nav_msgs::msg::OccupancyGrid::SharedPtr LayeredMapManager::getCostmap() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return costmap_;
}

nav_msgs::msg::OccupancyGrid::SharedPtr LayeredMapManager::getEsdfVis() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return esdf_vis_;
}

EsdfMap::Ptr LayeredMapManager::getEsdf() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return esdf_;
}

// ============ MapInterface实现 ============

void LayeredMapManager::getBounds(double& min_x, double& min_y,
                                   double& max_x, double& max_y) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    min_x = origin_x_;
    min_y = origin_y_;
    max_x = origin_x_ + width_ * resolution_;
    max_y = origin_y_ + height_ * resolution_;
}

nav_core::MapQuery LayeredMapManager::query(double x, double y) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    nav_core::MapQuery result;
    if (!costmap_) return result;
    
    int mx = static_cast<int>((x - origin_x_) / resolution_);
    int my = static_cast<int>((y - origin_y_) / resolution_);
    
    if (mx < 0 || mx >= width_ || my < 0 || my >= height_) {
        return result;
    }
    
    result.valid = true;
    int8_t val = costmap_->data[my * width_ + mx];
    result.occupied = (val >= obstacle_threshold_);
    result.cost = (val >= 0) ? val / 100.0 : 1.0;
    
    // 如果有ESDF，填充距离信息
    if (esdf_) {
        result.distance = esdf_->getDistanceFast(mx, my);
    }
    return result;
}

double LayeredMapManager::getEsdfDistanceWithGradient(double x, double y,
                                                       double* gx, double* gy) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (!esdf_) {
        if (gx) *gx = 0;
        if (gy) *gy = 0;
        return 1e6;
    }
    
    double dist = esdf_->getDistanceInterp(x, y);
    
    if (gx && gy) {
        esdf_->getGradient(x, y, *gx, *gy);
    }
    
    return dist;
}

}  // namespace nav_components
