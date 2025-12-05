// nav_components/include/nav_components/map_manager.hpp
// 地图管理器 - 统一管理地图加载、膨胀、ESDF、查询

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_core/map_interface.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "nav_components/static_map_loader.hpp"
#include "nav_components/costmap_inflater.hpp"
#include "nav_components/esdf_map.hpp"

namespace nav_components {

class MapManager : public nav_core::MapInterface {
public:
    // 从文件加载地图并生成 costmap
    bool loadFromFile(const std::string& yaml_path, 
                      const InflationParams& params,
                      rclcpp::Logger logger) {
        raw_map_ = StaticMapLoader::load(yaml_path, logger);
        if (!raw_map_) return false;
        
        inflation_params_ = params;
        rebuildCostmap();
        
        RCLCPP_INFO(logger, "地图加载完成: %dx%d, 膨胀半径=%.2fm",
                    width_, height_, params.inflation_radius);
        return true;
    }
    
    // 从外部设置原始地图
    void setRawMap(nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        raw_map_ = map;
        if (map) {
            rebuildCostmap();
        }
    }
    
    // 设置膨胀参数（会触发重建）
    void setInflationParams(const InflationParams& params) {
        inflation_params_ = params;
        if (raw_map_) {
            rebuildCostmap();
        }
    }
    
    // 构建 ESDF（可选，按需调用）
    void buildEsdf(double vis_max_dist = 2.0) {
        if (!raw_map_) {
            return;
        }
        esdf_ = std::make_shared<EsdfMap>();
        esdf_->buildFromOccupancy(raw_map_);
        esdf_vis_ = esdf_->toOccupancyGrid(vis_max_dist);
    }
    
    // 获取各种地图用于发布
    nav_msgs::msg::OccupancyGrid::SharedPtr getRawMap() const { return raw_map_; }
    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmap() const { return costmap_; }
    nav_msgs::msg::OccupancyGrid::SharedPtr getEsdfVis() const { return esdf_vis_; }
    EsdfMap::Ptr getEsdf() const { return esdf_; }
    
    bool hasMap() const { return costmap_ != nullptr; }
    bool hasEsdf() const { return esdf_ != nullptr; }
    
    // MapInterface 实现（查询 costmap）
    nav_core::MapType type() const override { return nav_core::MapType::COSTMAP; }
    double resolution() const override { return resolution_; }
    
    void getBounds(double& min_x, double& min_y, 
                   double& max_x, double& max_y) const override {
        min_x = origin_x_;
        min_y = origin_y_;
        max_x = origin_x_ + width_ * resolution_;
        max_y = origin_y_ + height_ * resolution_;
    }
    
    nav_core::MapQuery query(double x, double y) const override {
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
        
        // 如果有 ESDF，填充距离信息
        if (esdf_) {
            result.distance = esdf_->getDistanceFast(mx, my);
        }
        return result;
    }
    
    bool hasDistance() const override { return esdf_ != nullptr; }
    
    // 获取 ESDF 梯度（轨迹优化用）
    bool getGradient(double x, double y, double& gx, double& gy) const {
        if (!esdf_) return false;
        return esdf_->getGradient(x, y, gx, gy);
    }
    
    // 获取 ESDF 距离和梯度（B样条优化用）
    // 返回距离值，通过指针返回梯度
    double getEsdfDistanceWithGradient(double x, double y, double* gx, double* gy) const {
        if (!esdf_) {
            if (gx) *gx = 0;
            if (gy) *gy = 0;
            return 1e6;  // 无 ESDF 返回大距离
        }
        
        // 获取距离
        double dist = esdf_->getDistanceInterp(x, y);
        
        // 获取梯度
        if (gx && gy) {
            esdf_->getGradient(x, y, *gx, *gy);
        }
        
        return dist;
    }
    
    void setObstacleThreshold(int8_t thresh) { obstacle_threshold_ = thresh; }

private:
    void rebuildCostmap() {
        costmap_ = CostmapInflater::inflate(raw_map_, inflation_params_);
        if (costmap_) {
            resolution_ = costmap_->info.resolution;
            origin_x_ = costmap_->info.origin.position.x;
            origin_y_ = costmap_->info.origin.position.y;
            width_ = costmap_->info.width;
            height_ = costmap_->info.height;
        }
    }
    
    nav_msgs::msg::OccupancyGrid::SharedPtr raw_map_;      // 原始地图
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;      // 膨胀后的 costmap
    nav_msgs::msg::OccupancyGrid::SharedPtr esdf_vis_;     // ESDF 可视化
    std::shared_ptr<EsdfMap> esdf_;
    
    InflationParams inflation_params_;
    double resolution_ = 0.05;
    double origin_x_ = 0, origin_y_ = 0;
    int width_ = 0, height_ = 0;
    int8_t obstacle_threshold_ = 50;
};

}  // namespace nav_components
