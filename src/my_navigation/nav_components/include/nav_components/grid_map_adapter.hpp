// nav_components/include/nav_components/grid_map_adapter.hpp
// OccupancyGrid 适配器 - 将 nav_msgs/OccupancyGrid 适配到 MapInterface

#pragma once
#include <nav_core/map_interface.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace nav_components {

class GridMapAdapter : public nav_core::MapInterface {
public:
    void setMap(nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        map_ = map;
        if (map_) {
            resolution_ = map_->info.resolution;
            origin_x_ = map_->info.origin.position.x;
            origin_y_ = map_->info.origin.position.y;
            width_ = map_->info.width;
            height_ = map_->info.height;
        }
    }
    
    nav_core::MapQuery query(double x, double y) const override {
        nav_core::MapQuery result;
        if (!map_) return result;
        
        int mx = static_cast<int>((x - origin_x_) / resolution_);
        int my = static_cast<int>((y - origin_y_) / resolution_);
        
        if (mx < 0 || mx >= width_ || my < 0 || my >= height_) {
            return result;  // valid=false
        }
        
        result.valid = true;
        int8_t val = map_->data[my * width_ + mx];
        result.occupied = (val > obstacle_threshold_ || val < 0);
        result.cost = (val >= 0) ? val / 100.0 : 1.0;
        return result;
    }
    
    double resolution() const override { return resolution_; }
    
    void getBounds(double& min_x, double& min_y, 
                   double& max_x, double& max_y) const override {
        min_x = origin_x_;
        min_y = origin_y_;
        max_x = origin_x_ + width_ * resolution_;
        max_y = origin_y_ + height_ * resolution_;
    }
    
    void setObstacleThreshold(int8_t thresh) { obstacle_threshold_ = thresh; }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    double resolution_ = 0.05;
    double origin_x_ = 0, origin_y_ = 0;
    int width_ = 0, height_ = 0;
    int8_t obstacle_threshold_ = 50;
};

}  // namespace nav_components
