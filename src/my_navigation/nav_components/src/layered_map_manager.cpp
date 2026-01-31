// nav_components/src/layered_map_manager.cpp
// 分层地图管理器实现

#include "nav_components/layered_map_manager.hpp"
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <chrono>

namespace nav_components {

LayeredMapManager::LayeredMapManager()
    : logger_(rclcpp::get_logger("layered_map_manager")) {}

void LayeredMapManager::initialize(rclcpp::Node* node,
                                    std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    node_ = node;
    tf_buffer_ = tf_buffer;
    logger_ = node_->get_logger();
    
    // 读取坐标系参数（使用 get_parameter_or 更稳健，避免依赖参数声明顺序）
    node_->get_parameter_or<std::string>("map_frame", map_frame_, map_frame_);
    node_->get_parameter_or<std::string>("odom_frame", odom_frame_, odom_frame_);

    // 读取性能日志开关（默认 false）
    node_->get_parameter_or<bool>("enable_performance_logging", enable_performance_logging_, enable_performance_logging_);
    
    RCLCPP_INFO(logger_, "LayeredMapManager initialized (map: %s, odom: %s, perf_log: %s)",
                map_frame_.c_str(), odom_frame_.c_str(), 
                enable_performance_logging_ ? "enabled" : "disabled");
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
    
    // 2. 重置融合地图为静态地图
    if (!fused_map_) {
        fused_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    }
    *fused_map_ = *static_map_;
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
            
            // 融合策略：取max（保守，障碍物优先）
            int8_t static_val = static_map_->data[global_idx];
            int8_t fused_val = std::max(static_val, local_val);
            
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
                    // 恢复为静态地图的值
                    if (fused_map_->data[idx] != static_map_->data[idx]) {
                        fused_map_->data[idx] = static_map_->data[idx];
                        changed_cells.push_back(idx);
                    }
                }
            }
        }
    }
    
    // 更新边界记录
    last_dynamic_bounds_ = new_bounds;
    
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
